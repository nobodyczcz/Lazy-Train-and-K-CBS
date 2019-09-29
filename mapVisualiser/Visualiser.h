#pragma once

#include <SFML/Graphics.hpp>
#include <vector>
#include <iterator>
#include <list>

using namespace std;

struct Cell{
    int color;
    
};

class Visualiser {
    public:
        Visualiser(bool* map,int raw,int col);
        void renderWindow();
        void setGreen(int loc);
        void setRed(int loc);
        void setOrange(int loc);
        void setWhite(int loc);
        void setBlack(int loc);
        void setColor(int loc,int r,int g, int b,float opacity=1);
        void colorConstraints(const std::vector < std::list< std::pair<int, int> > >* constraints);
        ~Visualiser();
    private:
        void renderingThread();
        sf::RectangleShape* gridMap;
        float maxWidth = 1024;
        float height=1024;
        float scale;
        int size;
        sf::RenderWindow* window;
        sf::Thread m_thread;
};