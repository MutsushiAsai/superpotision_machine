#include <SFML/Graphics.hpp>
#include <armadillo>
#include <boost/asio.hpp>
#include <boost/interprocess/ipc/message_queue.hpp>
#include <cmath>
#include <csignal>
#include <exception>
#include <iostream>
#include <thread>

#include "constants.h"
#include "midi.h"
#include "synthesize.h"

namespace spsynth = superposition::synthesize;
namespace spmidi = superposition::midi;
namespace bip = boost::interprocess;
using namespace arma;
using namespace boost;

sf::ConvexShape createStar(float radius, float xPos, float yPos) {
    sf::ConvexShape star;
    star.setPointCount(10);

    float innerRadius = radius * 0.5f;  // 内側の頂点の半径
    float angleStep = M_PI / 5.0f;      // 頂点間の角度（36度）

    for (int i = 0; i < 10; ++i) {
        float angle = i * angleStep;
        float currentRadius = (i % 2 == 0) ? radius : innerRadius;

        float x = currentRadius * std::cos(angle) + xPos;
        float y = currentRadius * std::sin(angle) + yPos;

        star.setPoint(i, sf::Vector2f(x, y));
    }

    star.setFillColor(sf::Color::Yellow);  // 星の色を黄色に設定
    return star;
}

std::function<void(int)> shutdown_handler;
void handle_shutdown(int signum) {
    shutdown_handler(signum);
}

int main(int argc, char** argv) {
    spmidi::MidiTask midi_task;
    auto midi_buffer = midi_task.message_buffer();

    spsynth::SynthesizeTask synthesize_task(midi_buffer);
    auto result_buffer = synthesize_task.result_buffer();

    shutdown_handler = [&midi_task, &synthesize_task](int signum) {
        midi_task.stop();
        synthesize_task.stop();
    };

    std::signal(SIGKILL, handle_shutdown);
    std::signal(SIGTERM, handle_shutdown);

    midi_task.start();
    synthesize_task.start();

    bool isFullscreen = true;
    sf::VideoMode desktopMode = sf::VideoMode::getDesktopMode();

    sf::RenderWindow window(desktopMode, "SFML Star", sf::Style::Fullscreen);

    auto window_size = window.getSize();
    sf::ConvexShape star =
        createStar(100.0f, window_size.x / 2, window_size.y / 2);

    std::function<void()> draw = [&window]() {
        auto window_size = window.getSize();
        sf::ConvexShape star =
            createStar(100.0f, window_size.x / 2, window_size.y / 2);

        window.clear();
        window.draw(star);
        window.display();
    };

    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }

            if (event.type == sf::Event::KeyPressed &&
                event.key.code == sf::Keyboard::Escape) {
                window.close();

                if (isFullscreen) {
                    window.create(sf::VideoMode(800, 600), "",
                                  sf::Style::Titlebar | sf::Style::Close);
                } else {
                    window.create(desktopMode, "", sf::Style::Fullscreen);
                }
                isFullscreen = !isFullscreen;  // フラグを反転
            }

            draw();
        }
    }

    midi_task.stop();
    synthesize_task.stop();

    return 0;
}