#include "synthesize.h"

#include <armadillo>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <exception>
#include <iomanip>

#include "constants.h"
#include "midi.h"

namespace spmidi = superposition::midi;
namespace bip = boost::interprocess;
using namespace arma;

namespace superposition::synthesize {

const arma::vec T = linspace<vec>(0.0, 2.0 * 2.0 * datum::pi, SAMPLING_RATE + 1)
                        .head(SAMPLING_RATE);

inline arma::vec mod(arma::vec x, double y) {
    return x - floor(x / y) * y;
}

inline double map_range(double value, double in_min, double in_max,
                        double out_min, double out_max) {
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

vec fold(arma::vec value, double vmin, double vmax) {
    auto range_v = vmax - vmin;
    auto fv = mod(value - vmin, 2 * range_v);
    for (uword i = 0; i < value.size(); i++) {
        auto v = fv[i];
        fv[i] = (v > range_v) ? 2 * range_v - v : v;
    }
    return fv + vmin;
}

arma::vec sawtooth(arma::vec t, double width = 1.0, double offset = 1.0) {
    double pi = datum::pi;
    double twopi = 2 * pi;

    arma::vec tmod = mod(t, twopi);
    for (uword i = 0; i < t.size(); i++) {
        auto v = tmod[i];
        if (v < width * twopi) {
            tmod[i] = v / (pi * width) - 1.0;
        } else {
            tmod[i] = (pi * (1.0 + width) - v) / (pi * (1.0 - width));
        }
    }

    return (tmod + offset) * 0.5;
}

std::uint64_t xor_hash(double* arr, std::size_t size) {
    std::uint64_t hash = 0;
    for (std::size_t i = 0; i < size; ++i) {
        std::uint64_t value = *reinterpret_cast<std::uint64_t*>(&arr[i]);
        hash ^= value;
    }

    return hash;
}

void synthesize(arma::vec& t, PolygonalSynthesizerParams& param,
                PolygonalSynthesizerResult& result) {
    double _frequency = param.frequency;
    double _roll = param.roll;
    double _n = param.n;
    double _teeth = param.teeth;
    double _fold_ratio = param.fold_ratio;
    double _fm_ratio = param.fm_ratio;
    double _fm_modulation = param.fm_modulation;

    double theta_zero = datum::pi / _n;
    double T = datum::pi * (_n - 2) / (2 * _n) * _teeth;

    arma::vec in1 = t * _frequency;
    arma::vec roll = t * _roll;
    arma::vec phi =
        2.0 * datum::pi * sawtooth(in1 + _fm_modulation * sin(_fm_ratio * in1));
    arma::vec rotate = 2 * datum::pi * sawtooth(roll);
    arma::vec theta = 2 * theta_zero * mod(phi * _n / 2 * datum::pi, 1);
    arma::vec p = cos(theta_zero + T) / cos(theta - theta_zero + T);

    // use "%" multiply operator instead of "*"
    result.out1 = fold(_fold_ratio * (p % cos(phi + rotate)), -1, 1);
    result.out2 = fold(_fold_ratio * (p % sin(phi + rotate)), -1, 1);
}

void SynthesizeTask::process() {
    std::mutex mutex;
    PolygonalSynthesizerParams params;
    common::MessageBuffer<PolygonalSynthesizerParams> param_buffer(32);

    asio::io_context scheduler_io;
    std::uint64_t last_hash = 0;
    common::SimpleScheduler scheduler(
        scheduler_io, 80, [this, &params, &mutex, &param_buffer, &last_hash]() {
            std::lock_guard<std::mutex> lock(mutex);
            param_buffer.push(params);
            // std::cout << "fq: " << params.frequency << " ";
            // std::cout << "roll: " << params.roll << " ";
            // std::cout << "N: " << params.n << " ";
            // std::cout << "teeth: " << params.teeth << " ";
            // std::cout << "fold_ratio: " << params.fold_ratio << " ";
            // std::cout << "fm_ratio: " << params.fm_ratio << " ";
            // std::cout << "fm_modulation: " << params.fm_modulation << " "
            //           << std::endl;
        });
    scheduler.start();

    std::thread scheduler_thread([&scheduler_io]() { scheduler_io.run(); });
    auto receiver_thread = std::thread([this, &params, &mutex] {
        spmidi::ControlChange cc;
        while (_is_started.load()) {
            if (!_midi_buffer->pop(cc)) {
                continue;
            }

            mutex.lock();
            if (cc.channel == 0) {
                params.frequency = map_range(static_cast<double>(cc.value), 0.0,
                                             127.0, 0, 3000.0);
            } else if (cc.channel == 1) {
                params.roll = map_range(static_cast<double>(cc.value), 0.0,
                                        127.0, 0.1, 200.0);
            } else if (cc.channel == 2) {
                params.n = map_range(static_cast<double>(cc.value), 0.0, 127.0,
                                     2.1, 150.0);
            } else if (cc.channel == 3) {
                params.teeth = map_range(static_cast<double>(cc.value), 0.0,
                                         127.0, 0.01, 0.99);
            } else if (cc.channel == 4) {
                params.fold_ratio = map_range(static_cast<double>(cc.value),
                                              0.0, 127.0, 0.1, 100.0);
            } else if (cc.channel == 5) {
                params.fm_ratio = map_range(static_cast<double>(cc.value), 0.0,
                                            127.0, 0.01, 10.0);
            } else if (cc.channel == 6) {
                params.fm_modulation = map_range(static_cast<double>(cc.value),
                                                 0.0, 127.0, 0.1, 100.0);
            }

            mutex.unlock();
        }
    });

    // ceils result "forSAMPLING_RATE / FRAMESIZE"
    PolygonalSynthesizerResult result;
    int last_idx = int(((SAMPLING_RATE + FRAME_SIZE - 1) / FRAME_SIZE) - 1);
    PolygonalSynthesizerParams received_param;
    for (int next_idx = 0, cur_idx = 0; _is_started.load();) {
        if (!param_buffer.pop(received_param)) {
            continue;
        }

        next_idx = cur_idx + 1;

        arma::vec t =
            T(arma::span(cur_idx * FRAME_SIZE, next_idx * FRAME_SIZE - 1));
        synthesize(t, received_param, result);
        _result_buffer.push(result);

        cur_idx = (next_idx == last_idx) ? 0 : cur_idx + 1;
    }
    try {
        receiver_thread.join();
        scheduler_io.stop();
        scheduler_thread.join();
    } catch (std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
    }
}

}  // namespace superposition::synthesize