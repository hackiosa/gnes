/*
 * Copyright (C) 2014 Frederic Meyer
 * 
 * This file is part of gnes.
 *
 * gnes is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *   
 * gnes is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with gnes.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdio.h>
#include <SDL/SDL.h>
#include "6502.h"
#include "mmu.h"

int main(int argc, char** argv)
{
    SDL_Surface* screen = NULL;
    SDL_Event event;
    bool running = true;

    if (SDL_Init(SDL_INIT_EVERYTHING) != 0)
    {
        printf("SDL_Init Error: %s\n", SDL_GetError());
        return 1;
    }

    screen = SDL_SetVideoMode(256, 240, 32, SDL_SWSURFACE);
    if (screen == NULL)
    {
        printf("SDL_SetVideoMode Error: %s\n", SDL_GetError());
        SDL_Quit();
    }

    SDL_WM_SetCaption("gnes", "gnes");

    puts("Welcome to gnes, the free NES emulator.");

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
                running = false;
        }
    }

    SDL_FreeSurface(screen);

    return 0;
}
