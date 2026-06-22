#include "GpuGrass.h"

#include <QOpenGLExtraFunctions>
#include <QVector2D>
#include <QtGlobal>

#include <cmath>
#include <vector>

// GL 4.3 enums (may be absent from the ES-oriented extra-functions headers).
#ifndef GL_SHADER_STORAGE_BUFFER
#define GL_SHADER_STORAGE_BUFFER 0x90D2
#endif
#ifndef GL_ATOMIC_COUNTER_BUFFER
#define GL_ATOMIC_COUNTER_BUFFER 0x92C0
#endif
#ifndef GL_DRAW_INDIRECT_BUFFER
#define GL_DRAW_INDIRECT_BUFFER 0x8F3F
#endif
#ifndef GL_SHADER_STORAGE_BARRIER_BIT
#define GL_SHADER_STORAGE_BARRIER_BIT 0x00002000
#endif
#ifndef GL_ATOMIC_COUNTER_BARRIER_BIT
#define GL_ATOMIC_COUNTER_BARRIER_BIT 0x00001000
#endif
#ifndef GL_COMMAND_BARRIER_BIT
#define GL_COMMAND_BARRIER_BIT 0x00000040
#endif
#ifndef GL_ALL_BARRIER_BITS
#define GL_ALL_BARRIER_BITS 0xFFFFFFFF
#endif
#ifndef GL_COPY_READ_BUFFER
#define GL_COPY_READ_BUFFER 0x8F36
#endif
#ifndef GL_COPY_WRITE_BUFFER
#define GL_COPY_WRITE_BUFFER 0x8F37
#endif
#ifndef GL_MAP_READ_BIT
#define GL_MAP_READ_BIT 0x0001
#endif

namespace vsim {
namespace {

// Compute: generate blades around the camera. The terrain noise is ported
// verbatim from procgen::TerrainField so blades sit on the same surface.
const char* kCompute = R"GLSL(
#version 430 core
layout(local_size_x = 16, local_size_y = 16) in;
struct Blade { vec4 posyaw; vec4 hf; vec4 tint; };
layout(std430, binding = 0) buffer Blades { Blade blades[]; };
layout(binding = 1) uniform atomic_uint instanceCount;  // dedicated counter buffer
uniform uint u_seed, u_maxBlades;
uniform float u_heightM, u_featureM, u_macroM, u_lacunarity, u_gain, u_mountainMix;
uniform int u_octaves, u_G;
uniform float u_grassMaxFrac, u_slopeLo, u_slopeHi, u_heightMean, u_heightStd, u_flowerFrac;
uniform vec2 u_origin; uniform float u_cell;
uniform vec3 u_camPos; uniform mat4 u_vp;
uniform float u_falloffStart, u_falloffEnd;
uint hash32(uint x){ x^=x>>16; x*=0x7feb352du; x^=x>>15; x*=0x846ca68bu; x^=x>>16; return x; }
uint hash2(int ix,int iy,uint seed){ uint h=uint(ix)*0x9e3779b1u; h^=uint(iy)*0x85ebca77u; h^=seed*0xc2b2ae3du; return hash32(h); }
void cornerGrad(int ix,int iy,uint seed,out float gx,out float gy){ uint h=hash2(ix,iy,seed); float a=(float(h)/4294967296.0)*6.2831853; gx=cos(a); gy=sin(a); }
float fade(float t){ return t*t*t*(t*(t*6.0-15.0)+10.0); }
float gradient(float x,float y,uint seed){
  int x0=int(floor(x)),y0=int(floor(y)),x1=x0+1,y1=y0+1;
  float fx=x-float(x0),fy=y-float(y0);
  float a00x,a00y,a10x,a10y,a01x,a01y,a11x,a11y;
  cornerGrad(x0,y0,seed,a00x,a00y); cornerGrad(x1,y0,seed,a10x,a10y);
  cornerGrad(x0,y1,seed,a01x,a01y); cornerGrad(x1,y1,seed,a11x,a11y);
  float n00=a00x*fx+a00y*fy, n10=a10x*(fx-1.0)+a10y*fy, n01=a01x*fx+a01y*(fy-1.0), n11=a11x*(fx-1.0)+a11y*(fy-1.0);
  float u=fade(fx),v=fade(fy);
  return mix(mix(n00,n10,u),mix(n01,n11,u),v)*1.4142136;
}
float fbm(float x,float y,uint seed,int oct,float lac,float gn){ float s=0.0,a=1.0,f=1.0,n=0.0; for(int o=0;o<oct;++o){ s+=a*gradient(x*f,y*f,seed); n+=a; a*=gn; f*=lac; } return n>0.0?s/n:0.0; }
float ridged(float x,float y,uint seed,int oct,float lac,float gn){ float s=0.0,a=1.0,f=1.0,n=0.0; for(int o=0;o<oct;++o){ float r=1.0-abs(gradient(x*f,y*f,seed)); r*=r; s+=a*r; n+=a; a*=gn; f*=lac; } return n>0.0?s/n:0.0; }
float sstep(float e0,float e1,float x){ float t=clamp((x-e0)/(e1-e0),0.0,1.0); return t*t*(3.0-2.0*t); }
float terrainHeight(float wx,float wy){
  uint sMacro=u_seed^0x68bc21ebu, sWarp=u_seed^0xb5297a4du, sHills=u_seed, sMtn=u_seed^0x9e3779b9u;
  float macroF=1.0/u_macroM, baseF=1.0/u_featureM;
  float macro=fbm(wx*macroF,wy*macroF,sMacro,3,2.0,0.5)*0.5+0.5;
  float ma=sstep(0.40,0.68,macro);
  float nx=wx*baseF, ny=wy*baseF;
  float wxw=nx+0.9*fbm(nx*0.5,ny*0.5,sWarp,3,2.0,0.5);
  float wyw=ny+0.9*fbm(nx*0.5+5.2,ny*0.5+1.3,sWarp,3,2.0,0.5);
  float roll=fbm(wxw,wyw,sHills,u_octaves,u_lacunarity,u_gain);
  float ru=roll*0.5+0.5;
  int ro=min(u_octaves,4);
  float ridge=ridged(wxw,wyw,sMtn,ro,u_lacunarity,0.55);
  float mu=pow(clamp(ridge,0.0,1.0),0.7);
  float elev=0.10*ru+ma*(u_mountainMix*mu+0.35*ru);
  return clamp(elev,0.0,1.0)*u_heightM;
}
float rnd(uint h){ return float(h & 0xffffffu)/16777216.0; }
void main(){
  uvec2 id=gl_GlobalInvocationID.xy;
  if(id.x>=uint(u_G)||id.y>=uint(u_G)) return;
  int cx=int(id.x)-u_G/2, cy=int(id.y)-u_G/2;
  float baseX=u_origin.x+float(cx)*u_cell, baseY=u_origin.y+float(cy)*u_cell;
  uint hc=hash2(cx,cy,u_seed^0x1234567u);
  float wx=baseX+(rnd(hc)-0.5)*0.9*u_cell, wy=baseY+(rnd(hc*0x9e37u+1u)-0.5)*0.9*u_cell;
  float h=terrainHeight(wx,wy);
  float e=u_cell;
  float dhdx=(terrainHeight(wx+e,wy)-terrainHeight(wx-e,wy))/(2.0*e);
  float dhdy=(terrainHeight(wx,wy+e)-terrainHeight(wx,wy-e))/(2.0*e);
  float flatn=1.0/sqrt(dhdx*dhdx+dhdy*dhdy+1.0);
  float t=h/(u_heightM+1e-3);
  float density=(1.0-sstep(u_grassMaxFrac*0.55,u_grassMaxFrac,t))*sstep(u_slopeLo,u_slopeHi,flatn);
  float dist=length(vec2(wx,wy)-u_camPos.xy);
  float keep=density*(1.0-sstep(u_falloffStart,u_falloffEnd,dist));
  if(keep<=0.0) return;
  if(rnd(hc*0x85ebu+3u)>keep) return;
  vec4 clip=u_vp*vec4(wx,wy,-h,1.0);
  if(clip.w<=0.0) return;
  vec3 ndc=clip.xyz/clip.w;
  if(ndc.x<-1.3||ndc.x>1.3||ndc.y<-1.3||ndc.y>1.3||ndc.z>1.0) return;
  float reg=2.0*(sin(baseX*0.035)+cos(baseY*0.028));
  float yaw=reg+(rnd(hc*0x27d4u+4u)-0.5)*2.0;
  float u1=max(rnd(hc*0x165667u+5u),1e-6), u2=rnd(hc*0x2545f4u+6u);
  float height=max(0.03,u_heightMean+u_heightStd*sqrt(-2.0*log(u1))*cos(6.2831853*u2));
  bool flower=rnd(hc*0x1b873u+7u)<u_flowerFrac;
  vec3 tint;
  if(flower){ float fh=rnd(hc*0x3a5fu+8u); tint=fh<0.40?vec3(0.95,0.95,0.97):(fh<0.72?vec3(0.93,0.84,0.28):vec3(0.86,0.34,0.30)); }
  else { float v=0.85+0.34*rnd(hc*0x9f3bu+9u); tint=vec3((0.30+0.12*rnd(hc*0xc2b2u+10u))*v,0.52*v,0.18*v); }
  uint idx=atomicCounterIncrement(instanceCount);
  if(idx>=u_maxBlades) return;
  blades[idx].posyaw=vec4(wx,wy,-h,yaw);
  blades[idx].hf=vec4(height,flower?1.0:0.0,0.0,0.0);
  blades[idx].tint=vec4(tint,0.0);
}
)GLSL";

const char* kVert = R"GLSL(
#version 430 core
layout(location=0) in vec3 a_local;
layout(location=1) in vec3 a_normal;
layout(location=2) in vec4 i_posyaw;   // per-instance (from the compute-written buffer)
layout(location=3) in vec4 i_hf;
layout(location=4) in vec4 i_tint;
uniform mat4 u_vp; uniform vec3 u_campos; uniform float u_time, u_fadestart, u_fadeend;
out vec3 v_color; out vec3 v_world; out vec3 v_normal; out float v_hf; out float v_flower;
void main(){
  vec3 ipos=i_posyaw.xyz; float yaw=i_posyaw.w, height=i_hf.x, flower=i_hf.y;
  float hf=-a_local.z;
  float d=length(ipos-u_campos);
  height*=1.0-clamp((d-u_fadestart)/max(u_fadeend-u_fadestart,1.0),0.0,1.0);
  vec3 L=a_local*height; L.xy*=(1.0+flower*hf*hf*2.0);
  float s=sin(yaw),c=cos(yaw);
  vec3 r=vec3(c*L.x-s*L.y, s*L.x+c*L.y, L.z);
  float w=sin(u_time*1.6+ipos.x*0.22+ipos.y*0.18);
  vec3 world=ipos+vec3(r.xy+vec2(0.80,0.55)*(w*0.14*height*hf*hf), r.z);
  v_world=world; v_color=i_tint.xyz; v_hf=hf; v_flower=flower;
  v_normal=vec3(c*a_normal.x-s*a_normal.y, s*a_normal.x+c*a_normal.y, a_normal.z);
  gl_Position=u_vp*vec4(world,1.0);
}
)GLSL";

const char* kFrag = R"GLSL(
#version 430 core
in vec3 v_color; in vec3 v_world; in vec3 v_normal; in float v_hf; in float v_flower;
out vec4 o_color;
uniform vec3 u_campos, u_sundir; uniform float u_fogdensity, u_fogstart;
vec3 skyColor(vec3 dir){
  float up=-dir.z;
  vec3 z=vec3(0.15,0.35,0.66),h=vec3(0.80,0.86,0.93),g=vec3(0.16,0.18,0.22);
  return (up>=0.0)?mix(h,z,pow(clamp(up,0.0,1.0),0.45)):mix(h,g,clamp(-up*2.5,0.0,1.0));
}
void main(){
  vec3 stem=vec3(0.28,0.46,0.18);
  vec3 albedo=mix(v_color, mix(stem,v_color,smoothstep(0.6,0.95,v_hf)), v_flower);
  vec3 n=normalize(v_normal);
  float ndl=max(dot(n,normalize(u_sundir)),0.0);
  float hemi=0.5+0.5*(-n.z);
  vec3 ambient=mix(vec3(0.22,0.24,0.28),vec3(0.50,0.53,0.58),clamp(hemi,0.0,1.0));
  float ao=mix(0.55,1.0,v_hf);
  vec3 col=albedo*(ambient+vec3(0.85)*ndl)*ao;
  vec3 toFrag=v_world-u_campos; float dist=length(toFrag);
  vec3 vdir=dist>1e-4?toFrag/dist:vec3(0.0,0.0,1.0);
  float trans=pow(max(dot(vdir,normalize(u_sundir)),0.0),4.0);
  col+=albedo*trans*(0.25+0.75*v_hf)*0.8;
  float fd=max(dist-u_fogstart,0.0)*u_fogdensity;
  float fog=1.0-exp(-fd*fd);
  o_color=vec4(mix(col,skyColor(vdir),clamp(fog,0.0,1.0)),1.0);
}
)GLSL";

}  // namespace

GpuGrass::~GpuGrass() = default;

bool GpuGrass::init(QOpenGLExtraFunctions* gl) {
  if (!comp_.addShaderFromSourceCode(QOpenGLShader::Compute, kCompute) ||
      !comp_.link()) {
    qInfo("[GpuGrass] compute unavailable -> CPU flora fallback. log:\n%s",
          comp_.log().toLocal8Bit().constData());
    return false;  // compute unsupported -> caller falls back to CPU flora
  }
  if (!draw_.addShaderFromSourceCode(QOpenGLShader::Vertex, kVert) ||
      !draw_.addShaderFromSourceCode(QOpenGLShader::Fragment, kFrag) ||
      !draw_.link()) {
    qInfo("[GpuGrass] draw program failed: %s",
          draw_.log().toLocal8Bit().constData());
    return false;
  }
  qInfo("[GpuGrass] ready (compute + indirect).");

  gl->glGenBuffers(1, &ssbo_);
  gl->glGenBuffers(1, &indirect_);
  gl->glGenBuffers(1, &counter_);

  // Allocate the instance buffer BEFORE wiring its vertex attributes (some
  // drivers ignore attribs pointing at an unallocated buffer).
  maxBlades_ = params_.grid * params_.grid;
  gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_);
  gl->glBufferData(GL_SHADER_STORAGE_BUFFER,
                   GLsizeiptr(maxBlades_) * 12 * sizeof(float), nullptr,
                   GL_DYNAMIC_DRAW);
  buildBlade(gl);

  gl->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counter_);
  const unsigned int zero = 0u;
  gl->glBufferData(GL_ATOMIC_COUNTER_BUFFER, sizeof(zero), &zero, GL_DYNAMIC_DRAW);

  gl->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_);
  const unsigned int cmd[4] = {0u, 0u, 0u, 0u};
  gl->glBufferData(GL_DRAW_INDIRECT_BUFFER, sizeof(cmd), cmd, GL_DYNAMIC_DRAW);
  ready_ = true;
  return true;
}

void GpuGrass::setParams(const Params& p) { params_ = p; }

void GpuGrass::buildBlade(QOpenGLExtraFunctions* gl) {
  // Bezier ribbon blade with per-vertex normals (single LOD; the GPU regenerates
  // every frame so a moderate vertex count is fine). Mirrors the CPU blade.
  const float wb = 0.05f;
  const float P0x = 0, P0z = 0, P1x = 0.14f, P1z = -0.55f, P2x = 0.42f, P2z = -1.0f;
  auto bez = [&](float t, float& x, float& z) {
    float u = 1 - t;
    x = u * u * P0x + 2 * u * t * P1x + t * t * P2x;
    z = u * u * P0z + 2 * u * t * P1z + t * t * P2z;
  };
  auto nrm = [&](float t, float& nx, float& nz) {
    float tx = 2 * (1 - t) * (P1x - P0x) + 2 * t * (P2x - P1x);
    float tz = 2 * (1 - t) * (P1z - P0z) + 2 * t * (P2z - P1z);
    float rx = -tz, rz = tx - 0.9f;
    float l = std::sqrt(rx * rx + rz * rz);
    nx = rx / l; nz = rz / l;
  };
  auto width = [&](float t) { return wb * (0.06f + 0.94f * std::pow(1 - t, 0.7f)); };
  const int kSeg = 4;
  std::vector<float> v;
  auto vert = [&](float x, float y, float z, float nx, float nz) {
    v.insert(v.end(), {x, y, z, nx, 0.0f, nz});
  };
  for (int s = 0; s < kSeg; ++s) {
    float t0 = float(s) / kSeg, t1 = float(s + 1) / kSeg;
    float x0, z0, x1, z1, n0x, n0z, n1x, n1z;
    bez(t0, x0, z0); bez(t1, x1, z1);
    nrm(t0, n0x, n0z); nrm(t1, n1x, n1z);
    float w0 = width(t0), w1 = width(t1);
    vert(x0, +w0, z0, n0x, n0z); vert(x0, -w0, z0, n0x, n0z); vert(x1, -w1, z1, n1x, n1z);
    vert(x0, +w0, z0, n0x, n0z); vert(x1, -w1, z1, n1x, n1z); vert(x1, +w1, z1, n1x, n1z);
  }
  bladeVerts_ = int(v.size() / 6);
  bladeVbo_.create();
  bladeVbo_.bind();
  bladeVbo_.allocate(v.data(), int(v.size() * sizeof(float)));
  vao_.create();
  vao_.bind();
  // Per-vertex blade geometry (divisor 0) from bladeVbo_.
  const int gstride = 6 * sizeof(float);
  gl->glEnableVertexAttribArray(0);
  gl->glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, gstride, nullptr);
  gl->glEnableVertexAttribArray(1);
  gl->glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, gstride,
                            reinterpret_cast<void*>(3 * sizeof(float)));
  bladeVbo_.release();
  // Per-instance blade data (divisor 1) read from the SAME buffer the compute
  // writes — but as vertex attributes, which avoids the (often unsupported)
  // SSBO-read-in-vertex-shader. Layout: 3 vec4 = 48 bytes per blade.
  gl->glBindBuffer(GL_ARRAY_BUFFER, ssbo_);
  const int istride = 12 * sizeof(float);
  gl->glEnableVertexAttribArray(2);
  gl->glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, istride, nullptr);
  gl->glVertexAttribDivisor(2, 1);
  gl->glEnableVertexAttribArray(3);
  gl->glVertexAttribPointer(3, 4, GL_FLOAT, GL_FALSE, istride,
                            reinterpret_cast<void*>(4 * sizeof(float)));
  gl->glVertexAttribDivisor(3, 1);
  gl->glEnableVertexAttribArray(4);
  gl->glVertexAttribPointer(4, 4, GL_FLOAT, GL_FALSE, istride,
                            reinterpret_cast<void*>(8 * sizeof(float)));
  gl->glVertexAttribDivisor(4, 1);
  gl->glBindBuffer(GL_ARRAY_BUFFER, 0);
  vao_.release();
}

void GpuGrass::render(QOpenGLExtraFunctions* gl, const QMatrix4x4& proj,
                      const QMatrix4x4& view, const QVector3D& camPos,
                      const QVector3D& sunDir, float time) {
  if (!ready_) return;

  // Grow the instance buffer if the grid was enlarged.
  const int need = params_.grid * params_.grid;
  if (need != maxBlades_) {
    maxBlades_ = need;
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER,
                     GLsizeiptr(maxBlades_) * 12 * sizeof(float), nullptr,
                     GL_DYNAMIC_DRAW);
  }

  const QMatrix4x4 vp = proj * view;
  const float cell = params_.cell;
  const QVector2D origin(std::floor(camPos.x() / cell) * cell,
                         std::floor(camPos.y() / cell) * cell);

  // Reset the indirect command {vertexCount, instanceCount=0, first=0, base=0}
  // and the atomic counter to 0.
  const unsigned int cmd[4] = {static_cast<unsigned int>(bladeVerts_), 0u, 0u, 0u};
  gl->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_);
  gl->glBufferSubData(GL_DRAW_INDIRECT_BUFFER, 0, sizeof(cmd), cmd);
  const unsigned int zero = 0u;
  gl->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counter_);
  gl->glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(zero), &zero);

  // --- generate ---
  comp_.bind();
  comp_.setUniformValue("u_seed", static_cast<GLuint>(params_.seed));
  comp_.setUniformValue("u_maxBlades", static_cast<GLuint>(maxBlades_));
  comp_.setUniformValue("u_heightM", params_.heightM);
  comp_.setUniformValue("u_featureM", params_.featureM);
  comp_.setUniformValue("u_macroM", params_.macroM);
  comp_.setUniformValue("u_octaves", params_.octaves);
  comp_.setUniformValue("u_lacunarity", params_.lacunarity);
  comp_.setUniformValue("u_gain", params_.gain);
  comp_.setUniformValue("u_mountainMix", params_.mountainMix);
  comp_.setUniformValue("u_grassMaxFrac", params_.grassMaxFrac);
  comp_.setUniformValue("u_slopeLo", params_.slopeLo);
  comp_.setUniformValue("u_slopeHi", params_.slopeHi);
  comp_.setUniformValue("u_heightMean", params_.heightMean);
  comp_.setUniformValue("u_heightStd", params_.heightStdDev);
  comp_.setUniformValue("u_flowerFrac", params_.flowerFrac);
  comp_.setUniformValue("u_origin", origin);
  comp_.setUniformValue("u_cell", cell);
  comp_.setUniformValue("u_G", params_.grid);
  comp_.setUniformValue("u_camPos", camPos);
  comp_.setUniformValue("u_vp", vp);
  comp_.setUniformValue("u_falloffStart", params_.falloffStart);
  comp_.setUniformValue("u_falloffEnd", params_.falloffEnd);
  gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_);
  gl->glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 1, counter_);
  const int groups = (params_.grid + 15) / 16;
  gl->glDispatchCompute(groups, groups, 1);
  gl->glMemoryBarrier(GL_ALL_BARRIER_BITS);
  comp_.release();

  // Copy the generated count into the indirect command's instanceCount (offset
  // 4). A dedicated counter buffer is far more portable than aliasing the
  // indirect buffer as an atomic-counter buffer.
  gl->glBindBuffer(GL_COPY_READ_BUFFER, counter_);
  gl->glBindBuffer(GL_COPY_WRITE_BUFFER, indirect_);
  gl->glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 4, 4);
  gl->glMemoryBarrier(GL_ALL_BARRIER_BITS);

  // Diagnostic: read back the blade count (once-per-frame stall; remove later).
  gl->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counter_);
  void* ptr = gl->glMapBufferRange(GL_ATOMIC_COUNTER_BUFFER, 0, 4, GL_MAP_READ_BIT);
  if (ptr) {
    lastCount_ = *static_cast<unsigned int*>(ptr);
    gl->glUnmapBuffer(GL_ATOMIC_COUNTER_BUFFER);
  }
  static int frame = 0;
  if ((frame++ % 90) == 0) {
    // Read blade[0] from the SSBO: if pos/height are real, the compute wrote the
    // buffer (so any blank screen is a draw bug); if all zero, the writes didn't
    // land.
    gl->glBindBuffer(GL_ARRAY_BUFFER, ssbo_);
    float b0[12] = {0};
    void* bp = gl->glMapBufferRange(GL_ARRAY_BUFFER, 0, 48, GL_MAP_READ_BIT);
    if (bp) {
      for (int i = 0; i < 12; ++i) b0[i] = static_cast<float*>(bp)[i];
      gl->glUnmapBuffer(GL_ARRAY_BUFFER);
    }
    qInfo("[GpuGrass] %u blades cam=%.0f,%.0f | blade0 pos=(%.1f,%.1f,%.1f) "
          "yaw=%.2f h=%.2f tint=(%.2f,%.2f,%.2f)",
          lastCount_, double(camPos.x()), double(camPos.y()), double(b0[0]),
          double(b0[1]), double(b0[2]), double(b0[3]), double(b0[4]),
          double(b0[8]), double(b0[9]), double(b0[10]));
  }

  // --- draw ---
  while (gl->glGetError() != 0u) {}  // clear pending errors
  draw_.bind();
  draw_.setUniformValue("u_vp", vp);
  draw_.setUniformValue("u_campos", camPos);
  draw_.setUniformValue("u_time", time);
  draw_.setUniformValue("u_fadestart", params_.falloffStart);
  draw_.setUniformValue("u_fadeend", params_.falloffEnd);
  draw_.setUniformValue("u_sundir", sunDir);
  draw_.setUniformValue("u_fogdensity", 1.0f / 420.0f);
  draw_.setUniformValue("u_fogstart", 45.0f);
  vao_.bind();  // per-instance blade data read as vertex attributes (no SSBO read)
  gl->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_);
  gl->glDrawArraysIndirect(GL_TRIANGLES, nullptr);
  const unsigned int err = gl->glGetError();
  vao_.release();
  draw_.release();
  static int dframe = 0;
  if (err != 0u && (dframe++ % 90) == 0)
    qInfo("[GpuGrass] DRAW gl error 0x%x (verts=%d instances=%u)", err,
          bladeVerts_, lastCount_);
}

}  // namespace vsim
