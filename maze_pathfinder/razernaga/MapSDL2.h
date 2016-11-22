//
// Created by myrddin on 06/11/16.
//

#ifndef RAZERNAGA_MAPSDL2_H
#define RAZERNAGA_MAPSDL2_H

#include <vector>
#include <SDL2/SDL.h>

class MapSDL2 {
public:
    MapSDL2(int rows, int cols, int square_precision, int square_size);
    ~MapSDL2();

    void set_color(int x, int y, Uint8 R, Uint8 G, Uint8 B, Uint8 A);
    void render_full_map();
    std::vector<int> get_color(const int& x, const int& y) const;
    bool exit_requested();
private:
    typedef struct color {
        color() : R(220), G(220), B(220), A(255) {};
        Uint8 R, G, B, A;
    } Color;

    const int square_size_, square_precision_, rows_, cols_;

    SDL_Window* window_;
    SDL_Renderer* renderer_;
    SDL_Event events_;

    std::vector<std::vector<Color>> map_;
};

#endif //RAZERNAGA_MAPSDL2_H
