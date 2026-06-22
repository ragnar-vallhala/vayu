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
#ifndef GL_R32F
#define GL_R32F 0x822E
#endif
#ifndef GL_READ_ONLY
#define GL_READ_ONLY 0x88B8
#endif
#ifndef GL_WRITE_ONLY
#define GL_WRITE_ONLY 0x88B9
#endif

namespace vsim {
namespace {

// Compute: generate blades around the camera. The terrain noise is ported
// verbatim from procgen::TerrainField so blades sit on the same surface.

// Pass 1: evaluate the terrain noise ONCE per texel into a height image. The
// grass pass then samples this instead of computing noise per blade — the GoT
// height-texture trick, ~10x fewer noise evals per frame.
const char* kFill = R"GLSL(
#version 430 core
layout(local_size_x=16, local_size_y=16) in;
layout(r32f, binding=0) writeonly uniform image2D heightImg;
uniform uint u_seed;
uniform float u_heightM, u_featureM, u_macroM, u_lacunarity, u_gain, u_mountainMix;
uniform int u_octaves, u_texSize;
uniform vec2 u_regionMin; uniform float u_regionSize;
uint hash32(uint x){ x^=x>>16; x*=0x7feb352du; x^=x>>15; x*=0x846ca68bu; x^=x>>16; return x; }
uint hash2(int ix,int iy,uint seed){ uint h=uint(ix)*0x9e3779b1u; h^=uint(iy)*0x85ebca77u; h^=seed*0xc2b2ae3du; return hash32(h); }
void cg(int ix,int iy,uint seed,out float gx,out float gy){ uint h=hash2(ix,iy,seed); float a=(float(h)/4294967296.0)*6.2831853; gx=cos(a); gy=sin(a); }
float fade(float t){ return t*t*t*(t*(t*6.0-15.0)+10.0); }
float grad(float x,float y,uint seed){ int x0=int(floor(x)),y0=int(floor(y)),x1=x0+1,y1=y0+1; float fx=x-float(x0),fy=y-float(y0);
  float a,b,c,d,e,f,g,h2; cg(x0,y0,seed,a,b); cg(x1,y0,seed,c,d); cg(x0,y1,seed,e,f); cg(x1,y1,seed,g,h2);
  float n00=a*fx+b*fy,n10=c*(fx-1.0)+d*fy,n01=e*fx+f*(fy-1.0),n11=g*(fx-1.0)+h2*(fy-1.0);
  float u=fade(fx),v=fade(fy); return mix(mix(n00,n10,u),mix(n01,n11,u),v)*1.4142136; }
float fbm(float x,float y,uint s,int o,float l,float gn){ float su=0.,a=1.,fr=1.,n=0.; for(int i=0;i<o;++i){su+=a*grad(x*fr,y*fr,s);n+=a;a*=gn;fr*=l;} return n>0.?su/n:0.; }
float ridged(float x,float y,uint s,int o,float l,float gn){ float su=0.,a=1.,fr=1.,n=0.; for(int i=0;i<o;++i){float r=1.-abs(grad(x*fr,y*fr,s));r*=r;su+=a*r;n+=a;a*=gn;fr*=l;} return n>0.?su/n:0.; }
float ss(float e0,float e1,float x){ float t=clamp((x-e0)/(e1-e0),0.,1.); return t*t*(3.-2.*t); }
float terrainHeight(float wx,float wy){
  uint sM=u_seed^0x68bc21ebu,sW=u_seed^0xb5297a4du,sH=u_seed,sN=u_seed^0x9e3779b9u;
  float mF=1.0/u_macroM,bF=1.0/u_featureM;
  float ma=ss(0.40,0.68,fbm(wx*mF,wy*mF,sM,3,2.0,0.5)*0.5+0.5);
  float nx=wx*bF,ny=wy*bF;
  float wxw=nx+0.9*fbm(nx*0.5,ny*0.5,sW,3,2.0,0.5);
  float wyw=ny+0.9*fbm(nx*0.5+5.2,ny*0.5+1.3,sW,3,2.0,0.5);
  float ru=fbm(wxw,wyw,sH,u_octaves,u_lacunarity,u_gain)*0.5+0.5;
  float mu=pow(clamp(ridged(wxw,wyw,sN,min(u_octaves,4),u_lacunarity,0.55),0.,1.),0.7);
  return clamp(0.10*ru+ma*(u_mountainMix*mu+0.35*ru),0.,1.)*u_heightM;
}
void main(){
  ivec2 id=ivec2(gl_GlobalInvocationID.xy);
  if(id.x>=u_texSize||id.y>=u_texSize) return;
  vec2 wp=u_regionMin+((vec2(id)+0.5)/float(u_texSize))*u_regionSize;
  imageStore(heightImg, id, vec4(terrainHeight(wp.x,wp.y)));
}
)GLSL";

// Pass 2: place blades, sampling the height image (no per-blade noise).
const char* kCompute = R"GLSL(
#version 430 core
layout(local_size_x=16, local_size_y=16) in;
struct Blade { vec4 posyaw; vec4 hf; vec4 tint; };
layout(std430, binding=0) buffer Blades { Blade blades[]; };
layout(binding=1) uniform atomic_uint instanceCount;
layout(r32f, binding=0) readonly uniform image2D heightImg;
uniform uint u_seed; uniform float u_heightM;
uniform int u_G, u_texSize, u_maxBlades;
uniform float u_grassMaxFrac,u_slopeLo,u_slopeHi,u_heightMean,u_heightStd,u_flowerFrac;
uniform int u_originCellX,u_originCellY; uniform float u_cell;
uniform vec2 u_regionMin; uniform float u_regionSize;
uniform vec3 u_camPos; uniform mat4 u_vp;
uniform float u_falloffStart,u_falloffEnd,u_innerCut,u_heightScale;
uint hash32(uint x){ x^=x>>16; x*=0x7feb352du; x^=x>>15; x*=0x846ca68bu; x^=x>>16; return x; }
uint hash2(int ix,int iy,uint seed){ uint h=uint(ix)*0x9e3779b1u; h^=uint(iy)*0x85ebca77u; h^=seed*0xc2b2ae3du; return hash32(h); }
float rnd(uint h){ return float(h & 0xffffffu)/16777216.0; }
float ss(float e0,float e1,float x){ float t=clamp((x-e0)/(e1-e0),0.,1.); return t*t*(3.-2.*t); }
float sampleH(vec2 wp){
  vec2 tc=((wp-u_regionMin)/u_regionSize)*float(u_texSize)-0.5;
  ivec2 t0=ivec2(floor(tc)); vec2 f=tc-vec2(t0); ivec2 mx=ivec2(u_texSize-1);
  float h00=imageLoad(heightImg,clamp(t0,ivec2(0),mx)).r;
  float h10=imageLoad(heightImg,clamp(t0+ivec2(1,0),ivec2(0),mx)).r;
  float h01=imageLoad(heightImg,clamp(t0+ivec2(0,1),ivec2(0),mx)).r;
  float h11=imageLoad(heightImg,clamp(t0+ivec2(1,1),ivec2(0),mx)).r;
  return mix(mix(h00,h10,f.x),mix(h01,h11,f.x),f.y);
}
void main(){
  uvec2 id=gl_GlobalInvocationID.xy;
  if(id.x>=uint(u_G)||id.y>=uint(u_G)) return;
  int wcx=u_originCellX+int(id.x)-u_G/2, wcy=u_originCellY+int(id.y)-u_G/2;
  float baseX=float(wcx)*u_cell, baseY=float(wcy)*u_cell;
  uint hc=hash2(wcx,wcy,u_seed^0x1234567u);
  float wx=baseX+(rnd(hc)-0.5)*0.9*u_cell, wy=baseY+(rnd(hc*0x9e37u+1u)-0.5)*0.9*u_cell;
  float tm=u_regionSize/float(u_texSize);
  float h=sampleH(vec2(wx,wy));
  float dhdx=(sampleH(vec2(wx+tm,wy))-sampleH(vec2(wx-tm,wy)))/(2.0*tm);
  float dhdy=(sampleH(vec2(wx,wy+tm))-sampleH(vec2(wx,wy-tm)))/(2.0*tm);
  float flatn=1.0/sqrt(dhdx*dhdx+dhdy*dhdy+1.0);
  float t=h/(u_heightM+1e-3);
  float density=(1.0-ss(u_grassMaxFrac*0.55,u_grassMaxFrac,t))*ss(u_slopeLo,u_slopeHi,flatn);
  float dist=length(vec2(wx,wy)-u_camPos.xy);
  float outer=1.0-ss(u_falloffStart,u_falloffEnd,dist);
  float inner=(u_innerCut>0.0)?ss(u_innerCut*0.70,u_innerCut,dist):1.0;  // far ring skips the near zone
  float keep=density*outer*inner;
  if(keep<=0.0||rnd(hc*0x85ebu+3u)>keep) return;
  vec4 clip=u_vp*vec4(wx,wy,-h,1.0);
  if(clip.w<=0.0) return; vec3 ndc=clip.xyz/clip.w;
  if(ndc.x<-1.3||ndc.x>1.3||ndc.y<-1.3||ndc.y>1.3||ndc.z>1.0) return;
  uint idx=atomicCounterIncrement(instanceCount);
  if(idx>=uint(u_maxBlades)) return;  // never write past the instance buffer
  float yaw=2.0*(sin(baseX*0.035)+cos(baseY*0.028))+(rnd(hc*0x27d4u+4u)-0.5)*2.0;
  float u1=max(rnd(hc*0x165667u+5u),1e-6),u2=rnd(hc*0x2545f4u+6u);
  float height=u_heightScale*max(0.03,u_heightMean+u_heightStd*sqrt(-2.0*log(u1))*cos(6.2831853*u2));
  float bend=0.55+0.9*rnd(hc*0x51e3u+11u);  // per-blade lean (top-down coverage + variety)
  bool fl=rnd(hc*0x1b873u+7u)<u_flowerFrac; vec3 tint;
  if(fl){ float fh=rnd(hc*0x3a5fu+8u); tint=fh<0.45?vec3(0.93,0.83,0.30):(fh<0.78?vec3(0.90,0.55,0.62):vec3(0.95,0.95,0.97)); }
  else { float v=0.82+0.36*rnd(hc*0x9f3bu+9u); tint=vec3((0.28+0.12*rnd(hc*0xc2b2u+10u))*v,0.50*v,0.17*v); }
  blades[idx].posyaw=vec4(wx,wy,-h,yaw);
  blades[idx].hf=vec4(height,fl?1.0:0.0,bend,0.0);
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
  vec3 ipos=i_posyaw.xyz; float yaw=i_posyaw.w, height=i_hf.x, flower=i_hf.y, bend=i_hf.z;
  float hf=-a_local.z;
  float d=length(ipos-u_campos);
  height*=1.0-clamp((d-u_fadestart)/max(u_fadeend-u_fadestart,1.0),0.0,1.0);
  vec3 L=a_local*height; L.x*=bend; L.xy*=(1.0+flower*hf*hf*0.9);
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
  // Darker stem at the base, blade colour toward the tip; flowers brighten the top.
  vec3 stem=vec3(0.16,0.30,0.10);
  vec3 grassAlbedo=mix(stem,v_color,smoothstep(0.20,0.85,v_hf));
  vec3 albedo=mix(grassAlbedo, mix(grassAlbedo,v_color,smoothstep(0.55,0.98,v_hf)), v_flower);

  // Soften the per-vertex normal toward up — real grass scatters light, so a
  // pure ribbon normal reads too hard. Blend gives the soft GoT shading.
  vec3 up=vec3(0.0,0.0,-1.0);
  vec3 n=normalize(mix(normalize(v_normal), up, 0.40));
  vec3 sun=normalize(u_sundir);
  vec3 sunCol=vec3(1.10,0.98,0.80);   // warm key light

  // Wrapped diffuse — softens the terminator so blades don't go flat-black.
  float ndl=dot(n,sun);
  float wrap=clamp(ndl*0.5+0.5,0.0,1.0); wrap*=wrap;

  // Hemispheric sky ambient: cool blue from above, darker green bounce below.
  float hemi=clamp(0.5+0.5*(-n.z),0.0,1.0);
  vec3 ambient=mix(vec3(0.13,0.17,0.13),vec3(0.40,0.50,0.60),hemi);

  float ao=mix(0.40,1.0,v_hf);   // base is occluded by the canopy
  vec3 col=albedo*(ambient + sunCol*wrap*1.05)*ao;

  vec3 toFrag=v_world-u_campos; float dist=length(toFrag);
  vec3 vdir=dist>1e-4?toFrag/dist:vec3(0.0,0.0,1.0);

  // Subsurface translucency: thin blades glow when backlit (camera looking
  // toward the sun through the canopy). The signature GoT meadow look.
  float back=pow(max(dot(vdir,sun),0.0),3.0);
  col+=albedo*sunCol*back*(0.30+0.70*v_hf)*0.9;

  // Soft anisotropic sheen along the blades.
  vec3 hv=normalize(sun-vdir);
  float spec=pow(max(dot(n,hv),0.0),18.0)*0.22*v_hf;
  col+=sunCol*spec;
  // Tip highlight.
  col+=albedo*0.12*smoothstep(0.75,1.0,v_hf);

  float fd=max(dist-u_fogstart,0.0)*u_fogdensity;
  float fog=1.0-exp(-fd*fd);
  o_color=vec4(mix(col,skyColor(vdir),clamp(fog,0.0,1.0)),1.0);
}
)GLSL";

}  // namespace

// Concentric density rings share the pipeline: a dense near ring plus coarser
// rings (cell multiplied) that carry grass out to the visible horizon. All
// append to the same instance buffer, so it must hold this many grids' worth of
// blades. Each ring's cell = base cell * kRingMul; coarser rings get slightly
// taller blades so the sparse far field still reads as a carpet.
static constexpr int kNumRings = 3;
static constexpr float kRingMul[kNumRings] = {1.0f, 4.0f, 12.0f};
static constexpr float kRingHScale[kNumRings] = {1.0f, 1.15f, 1.32f};

GpuGrass::~GpuGrass() = default;

bool GpuGrass::init(QOpenGLExtraFunctions* gl) {
  if (!fill_.addShaderFromSourceCode(QOpenGLShader::Compute, kFill) ||
      !fill_.link() ||
      !comp_.addShaderFromSourceCode(QOpenGLShader::Compute, kCompute) ||
      !comp_.link()) {
    qInfo("[GpuGrass] compute unavailable -> CPU flora fallback. log:\n%s%s",
          fill_.log().toLocal8Bit().constData(),
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
  maxBlades_ = kNumRings * params_.grid * params_.grid;
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

  // R32F terrain-height image (immutable storage, point access via imageLoad).
  gl->glGenTextures(1, &heightTex_);
  gl->glBindTexture(GL_TEXTURE_2D, heightTex_);
  gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  gl->glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  gl->glTexStorage2D(GL_TEXTURE_2D, 1, GL_R32F, texSize_, texSize_);
  gl->glBindTexture(GL_TEXTURE_2D, 0);

  ready_ = true;
  return true;
}

void GpuGrass::setParams(const Params& p) { params_ = p; }

void GpuGrass::buildBlade(QOpenGLExtraFunctions* gl) {
  // Bezier ribbon blade with per-vertex normals (single LOD; the GPU regenerates
  // every frame so a moderate vertex count is fine). Mirrors the CPU blade.
  const float wb = 0.09f;  // base half-width (wider blades read less thin)
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
  const int need = kNumRings * params_.grid * params_.grid;
  if (need != maxBlades_) {
    maxBlades_ = need;
    gl->glBindBuffer(GL_SHADER_STORAGE_BUFFER, ssbo_);
    gl->glBufferData(GL_SHADER_STORAGE_BUFFER,
                     GLsizeiptr(maxBlades_) * 12 * sizeof(float), nullptr,
                     GL_DYNAMIC_DRAW);
  }

  const QMatrix4x4 vp = proj * view;

  // Reset the indirect command {vertexCount, instanceCount=0, first=0, base=0}
  // and the atomic counter to 0 — ONCE; both rings accumulate into them.
  const unsigned int cmd[4] = {static_cast<unsigned int>(bladeVerts_), 0u, 0u, 0u};
  gl->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_);
  gl->glBufferSubData(GL_DRAW_INDIRECT_BUFFER, 0, sizeof(cmd), cmd);
  const unsigned int zero = 0u;
  gl->glBindBuffer(GL_ATOMIC_COUNTER_BUFFER, counter_);
  gl->glBufferSubData(GL_ATOMIC_COUNTER_BUFFER, 0, sizeof(zero), &zero);

  // Concentric density rings: a dense near ring, then coarser rings that carry
  // grass to the horizon. Each fades smoothly before its grid edge; each outer
  // ring fades IN where the previous one fades out (innerCut) to avoid a doubled
  // band. All sample the SAME height texture, refilled per ring.
  struct Ring {
    float cell, innerCut, falloffStart, falloffEnd, heightScale;
  };
  Ring rings[kNumRings];
  float prevRadius = 0.0f;
  for (int i = 0; i < kNumRings; ++i) {
    const float c = params_.cell * kRingMul[i];
    const float radius = float(params_.grid) * c * 0.5f;
    rings[i] = {c, (i == 0) ? 0.0f : prevRadius * 0.70f, radius * 0.80f,
                radius * 0.97f, kRingHScale[i]};
    prevRadius = radius;
  }
  const float farRadius = prevRadius;  // outermost ring radius (for the draw fade)

  for (const Ring& ring : rings) {
    // Anchor the grid to world cells (not the camera) so the blade field stays
    // fixed and the camera moves through it.
    const int originCellX = int(std::floor(camPos.x() / ring.cell));
    const int originCellY = int(std::floor(camPos.y() / ring.cell));
    const int halfGrid = params_.grid / 2;  // grid origin offset, in cells
    const float regionSize = float(params_.grid) * ring.cell;
    const QVector2D regionMin(float(originCellX - halfGrid) * ring.cell,
                              float(originCellY - halfGrid) * ring.cell);

    // --- pass 1: fill the height texture (terrain noise once per texel) ---
    fill_.bind();
    gl->glUniform1ui(fill_.uniformLocation("u_seed"), params_.seed);
    fill_.setUniformValue("u_heightM", params_.heightM);
    fill_.setUniformValue("u_featureM", params_.featureM);
    fill_.setUniformValue("u_macroM", params_.macroM);
    fill_.setUniformValue("u_octaves", params_.octaves);
    fill_.setUniformValue("u_lacunarity", params_.lacunarity);
    fill_.setUniformValue("u_gain", params_.gain);
    fill_.setUniformValue("u_mountainMix", params_.mountainMix);
    fill_.setUniformValue("u_texSize", texSize_);
    fill_.setUniformValue("u_regionMin", regionMin);
    fill_.setUniformValue("u_regionSize", regionSize);
    gl->glBindImageTexture(0, heightTex_, 0, GL_FALSE, 0, GL_WRITE_ONLY, GL_R32F);
    const int fg = (texSize_ + 15) / 16;
    gl->glDispatchCompute(fg, fg, 1);
    gl->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    fill_.release();

    // --- pass 2: place blades (sampling the height texture, no per-blade noise) ---
    comp_.bind();
    // uint uniforms must go through glUniform1ui (Qt's GLuint overload no-ops here).
    gl->glUniform1ui(comp_.uniformLocation("u_seed"), params_.seed);
    comp_.setUniformValue("u_heightM", params_.heightM);
    comp_.setUniformValue("u_grassMaxFrac", params_.grassMaxFrac);
    comp_.setUniformValue("u_slopeLo", params_.slopeLo);
    comp_.setUniformValue("u_slopeHi", params_.slopeHi);
    comp_.setUniformValue("u_heightMean", params_.heightMean);
    comp_.setUniformValue("u_heightStd", params_.heightStdDev);
    comp_.setUniformValue("u_flowerFrac", params_.flowerFrac);
    comp_.setUniformValue("u_originCellX", originCellX);
    comp_.setUniformValue("u_originCellY", originCellY);
    comp_.setUniformValue("u_cell", ring.cell);
    comp_.setUniformValue("u_G", params_.grid);
    comp_.setUniformValue("u_texSize", texSize_);
    comp_.setUniformValue("u_maxBlades", maxBlades_);
    comp_.setUniformValue("u_regionMin", regionMin);
    comp_.setUniformValue("u_regionSize", regionSize);
    comp_.setUniformValue("u_camPos", camPos);
    comp_.setUniformValue("u_vp", vp);
    comp_.setUniformValue("u_falloffStart", ring.falloffStart);
    comp_.setUniformValue("u_falloffEnd", ring.falloffEnd);
    comp_.setUniformValue("u_innerCut", ring.innerCut);
    comp_.setUniformValue("u_heightScale", ring.heightScale);
    gl->glBindImageTexture(0, heightTex_, 0, GL_FALSE, 0, GL_READ_ONLY, GL_R32F);
    gl->glBindBufferBase(GL_SHADER_STORAGE_BUFFER, 0, ssbo_);
    gl->glBindBufferBase(GL_ATOMIC_COUNTER_BUFFER, 1, counter_);
    const int groups = (params_.grid + 15) / 16;
    gl->glDispatchCompute(groups, groups, 1);
    gl->glMemoryBarrier(GL_ALL_BARRIER_BITS);
    comp_.release();
  }

  // Copy the generated count into the indirect command's instanceCount (offset
  // 4). A dedicated counter buffer is far more portable than aliasing the
  // indirect buffer as an atomic-counter buffer.
  gl->glBindBuffer(GL_COPY_READ_BUFFER, counter_);
  gl->glBindBuffer(GL_COPY_WRITE_BUFFER, indirect_);
  gl->glCopyBufferSubData(GL_COPY_READ_BUFFER, GL_COPY_WRITE_BUFFER, 0, 4, 4);
  gl->glMemoryBarrier(GL_ALL_BARRIER_BITS);

  // --- draw ---
  draw_.bind();
  draw_.setUniformValue("u_vp", vp);
  draw_.setUniformValue("u_campos", camPos);
  draw_.setUniformValue("u_time", time);
  // Height fade only at the very far edge — near grass keeps full height.
  draw_.setUniformValue("u_fadestart", farRadius * 0.85f);
  draw_.setUniformValue("u_fadeend", farRadius * 0.99f);
  draw_.setUniformValue("u_sundir", sunDir);
  draw_.setUniformValue("u_fogdensity", 1.0f / 420.0f);
  draw_.setUniformValue("u_fogstart", 45.0f);
  vao_.bind();  // per-instance blade data read as vertex attributes (no SSBO read)
  gl->glBindBuffer(GL_DRAW_INDIRECT_BUFFER, indirect_);
  gl->glDrawArraysIndirect(GL_TRIANGLES, nullptr);
  vao_.release();
  draw_.release();
}

}  // namespace vsim
