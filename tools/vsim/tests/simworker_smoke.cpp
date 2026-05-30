// Headless smoke test for the Navigator-side SimWorker glue.
//
// Drives the REAL vsim::SimWorker (software/src/vsim/SimWorker.cpp): it
// spawns the vsim_d daemon via posix_spawnp, opens /tmp/vsim_pose +
// /tmp/vsim_ctl, decodes pose frames, and emits poseUpdated across the
// thread boundary. We count those signals and sanity-check the latest
// snapshot. No GUI, no OpenGL, no firmware -- just the supervisor glue
// the vsim_d split introduced.
//
// vsim_d path comes from $VSIM_BIN_PATH (the harness sets it).
//
// Exit 0 if a healthy pose stream arrived; non-zero otherwise.

#include "SimWorker.h"

#include <QCoreApplication>
#include <QTimer>
#include <atomic>
#include <cmath>
#include <cstdio>

int main(int argc, char** argv) {
    QCoreApplication app(argc, argv);

    vsim::SimWorker worker;

    std::atomic<int> count{0};
    vsim::SimSnapshot last;
    QObject::connect(&worker, &vsim::SimWorker::poseUpdated,
                     &app, [&](vsim::SimSnapshot s) {
                         last = s;
                         count.fetch_add(1, std::memory_order_relaxed);
                     });
    QObject::connect(&worker, &vsim::SimWorker::logLine, &app,
                     [](const QString& l) {
                         std::fprintf(stderr, "  [worker] %s\n",
                                      l.toLocal8Bit().constData());
                     });

    // Fire a reset ~0.6 s in to exercise the ctl FIFO write path too.
    QTimer::singleShot(600, &app, [&] { worker.sendReset(); });

    // ~0.9 s in, push a custom geometry (heavier, more top-heavy inertia)
    // to exercise VSIM_CTL_SET_GEOMETRY end-to-end. The daemon should log
    // "geometry set" and the pose stream must stay healthy (no desync).
    QTimer::singleShot(900, &app, [&] {
        vsim::GeometryConfig g;
        g.mass = 1.5f;
        g.inertia = {0.05f, 0, 0, 0, 0.05f, 0, 0, 0, 0.09f};
        worker.sendGeometry(g);
    });

    // Run for ~1.5 s of pose streaming, then stop + quit.
    QTimer::singleShot(1500, &app, [&] {
        worker.requestStop();
        worker.wait(2000);
        app.quit();
    });

    worker.start();
    app.exec();

    const int n = count.load();
    const float qn = std::sqrt(last.att.scalar() * last.att.scalar() +
                               last.att.x() * last.att.x() +
                               last.att.y() * last.att.y() +
                               last.att.z() * last.att.z());

    std::printf("poseUpdated count : %d\n", n);
    std::printf("last pos_w (NED)  : (%.3f, %.3f, %.3f)\n",
                last.pos_w.x(), last.pos_w.y(), last.pos_w.z());
    std::printf("last quat |q|     : %.4f\n", qn);
    std::printf("last tick_count   : %llu\n",
                static_cast<unsigned long long>(last.tick_count));

    if (n < 10) {
        std::printf("FAIL: too few pose updates (%d) -- spawn/FIFO/decode broken\n", n);
        return 2;
    }
    if (std::fabs(qn - 1.0f) > 1e-2f) {
        std::printf("FAIL: attitude not a unit quaternion\n");
        return 3;
    }
    if (last.tick_count == 0) {
        std::printf("FAIL: tick_count never advanced\n");
        return 4;
    }
    std::printf("PASS: SimWorker spawned vsim_d, decoded pose, emitted %d updates\n", n);
    return 0;
}
