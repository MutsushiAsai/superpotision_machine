#include <SFML/Audio.hpp>
#include <SFML/Graphics.hpp>
#include <algorithm>
#include <armadillo>
#include <boost/asio.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <cmath>
#include <csignal>
#include <exception>
#include <iostream>
#include <list>
#include <mutex>
#include <thread>

#include "constants.h"
#include "midi.h"
#include "synthesize.h"

namespace spsynth = superposition::synthesize;
namespace spmidi = superposition::midi;
namespace bip = boost::interprocess;
using namespace arma;
using namespace boost;

std::function<void(int)> shutdown_handler;
void handle_shutdown(int signum) {
    shutdown_handler(signum);
}

class SynthesizeSound : public sf::SoundStream {
   public:
    SynthesizeSound() : _sample_index(0) {
        initialize(2, SAMPLING_RATE);
    }

    void load_buffer(std::vector<arma::vec>& left_samples,
                     std::vector<arma::vec>& right_samples) {
        std::lock_guard<std::mutex> lock(_mutex);
        _left_samples = left_samples;
        _right_samples = right_samples;
        _sample_index = 0;
    }

   protected:
    virtual bool onGetData(Chunk& data) {
        std::lock_guard<std::mutex> lock(_mutex);
        const size_t chunk_size = 4410;
        _samples.resize(chunk_size * 2);

        for (size_t i = 0; i < chunk_size; i++, _sample_index++) {
            double l_sample = 0.0;
            double r_sample = 0.0;
            for (size_t ch = 0; ch < _left_samples.size(); ch++) {
                if (_sample_index < _left_samples[ch].size()) {
                    l_sample += _left_samples[ch][_sample_index];
                }
                if (_sample_index < _right_samples[ch].size()) {
                    r_sample += _right_samples[ch][_sample_index];
                }
            }

            _samples[i * 2] = static_cast<sf::Int16>(l_sample * 0x7FFF);
            _samples[i * 2 + 1] = static_cast<sf::Int16>(r_sample * 0x7FFF);
        }
        data.samples = _samples.data();
        data.sampleCount = _samples.size();

        return true;
    }

    virtual void onSeek(sf::Time time_offset) override {
        _sample_index = SAMPLING_RATE * time_offset.asMilliseconds() * 0.001;
        std::cout << "time_offset : " << time_offset.asMilliseconds()
                  << std::endl;
        std::cout << "sample_index: " << _sample_index << std::endl;
    }

   private:
    std::vector<arma::vec> _left_samples;
    std::vector<arma::vec> _right_samples;
    std::vector<sf::Int16> _samples;
    size_t _sample_index = 0;
    std::mutex _mutex;
};

class SynthesizeController {
   public:
    SynthesizeController(std::string device, unsigned int baudrate)
        : _midi_task(device, baudrate),
          _synthesize_task(_midi_task.message_buffer()) {
    }

    void start() {
        _midi_task.start();
        _synthesize_task.start();
    }

    void draw(sf::RenderTexture& texture) {
        std::lock_guard<std::mutex> lock(_mutex);
        auto result_buffer = _synthesize_task.result_buffer();
        if (!result_buffer->pop(_last_result)) {
            return;
        }

        sf::VertexArray points(sf::PrimitiveType::LineStrip, 441);
        auto x = _last_result.out1;
        auto y = _last_result.out2;

        size_t plot_size = points.getVertexCount();

        auto w_size = texture.getSize();

        double padding = 50.0f;
        double ploat_w = w_size.x - 2 * padding;
        double ploat_h = w_size.y - 2 * padding;

        double center_x = w_size.x / 2.0f;
        double center_y = w_size.y / 2.0f;

        for (size_t i = 0; i < plot_size; i++) {
            double scaled_x = center_x + (x[i] * ploat_w * .5);
            double scaled_y = center_y - (y[i] * ploat_h * .5);

            points[i].position = sf::Vector2f(scaled_x, scaled_y);
            points[i].color = sf::Color::White;
        }

        texture.clear(sf::Color::Transparent);
        texture.draw(points);
        texture.display();
    }

    void stop() {
        _midi_task.stop();
        _midi_task.join();

        _synthesize_task.stop();
        _synthesize_task.join();
    }

    spsynth::PolygonalSynthesizerResult last_result() {
        std::lock_guard<std::mutex> lock(_mutex);
        return _last_result;
    }

   private:
    spmidi::MidiTask _midi_task;
    spsynth::SynthesizeTask _synthesize_task;
    spsynth::PolygonalSynthesizerResult _last_result;
    std::mutex _mutex;
};

int main(int argc, char** argv) {
    SynthesizeController controller("/dev/ttyUSB0", 38400);

    shutdown_handler = [&controller](int signum) { controller.stop(); };

    std::signal(SIGKILL, handle_shutdown);
    std::signal(SIGTERM, handle_shutdown);

    controller.start();

    bool isFullscreen = true;
    sf::VideoMode desktopMode = sf::VideoMode::getDesktopMode();
    sf::RenderWindow window(desktopMode, "", sf::Style::Fullscreen);
    if (isFullscreen) {
        window.create(sf::VideoMode(800, 600), "",
                      sf::Style::Titlebar | sf::Style::Close);
    } else {
        window.create(desktopMode, "", sf::Style::Fullscreen);
    }
    sf::RenderTexture texture1;
    texture1.create(window.getSize().x, window.getSize().y);

    auto window_size = window.getSize();
    SynthesizeSound sound;
    std::vector<arma::vec> l_samples(4);
    std::vector<arma::vec> r_samples(4);
    sound.play();
    while (window.isOpen()) {
        l_samples.clear();
        r_samples.clear();

        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            } else if (event.type == sf::Event::KeyPressed &&
                       event.key.code == sf::Keyboard::Escape) {
                window.close();

                if (isFullscreen) {
                    window.create(sf::VideoMode(800, 600), "",
                                  sf::Style::Titlebar | sf::Style::Close);
                } else {
                    window.create(desktopMode, "", sf::Style::Fullscreen);
                }
                texture1.create(window.getSize().x, window.getSize().y);
                isFullscreen = !isFullscreen;
            }
        }

        controller.draw(texture1);
        l_samples.push_back(controller.last_result().out1);
        r_samples.push_back(controller.last_result().out2);

        window.clear();
        window.draw(sf::Sprite(texture1.getTexture()));
        window.display();

        sound.load_buffer(l_samples, r_samples);
    }

    controller.stop();

    return 0;
}