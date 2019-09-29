#include "Visualiser.h"


Visualiser::Visualiser(bool* map,int raw,int col ) : m_thread(&Visualiser::renderingThread, this) {
    //normalize the size of the map
    if(raw>=col){
        scale = maxWidth/raw;
    }
    else{
        scale = maxWidth/col;
    }
    size = raw*col;

    height = maxWidth*(raw/col);
    gridMap = new sf::RectangleShape[size];
    for(int i = 0;i<(size);i++){
        sf::RectangleShape rectangle(sf::Vector2f(1.f*scale, 1.f*scale));
        rectangle.setPosition(i/col*scale,i%col/scale);
        if (map[i]){
            rectangle.setFillColor(sf::Color(255,255,255));
        }
        else{
            rectangle.setFillColor(sf::Color(0,0,0));
        }
        gridMap[i] = rectangle;

    }

    window = new sf::RenderWindow(sf::VideoMode(maxWidth, height), "My window");
}

void Visualiser::renderWindow(){
    m_thread.launch();
}
void Visualiser::colorConstraints(const std::vector < std::list< std::pair<int, int> > >* cons){
    for (int timestep = 0;timestep<cons->size();timestep++){
        for (std::list< std::pair<int, int> >::const_iterator it = cons->at(timestep).begin(); it != cons->at(timestep).end(); ++it)
        {
            float opacity =0.5 + 0.5*(timestep/cons->size());
            if (std::get<1>(*it) < 0){
                setColor(std::get<0>(*it),255,0,0,opacity); 
            }
            else{
                setColor(std::get<0>(*it),204,0,102,opacity); 
            }
        }
    }

}

void Visualiser::renderingThread()
{
    // activate the window's context
    this->window->setActive(true);

    // the rendering loop
    while (this->window->isOpen())
    {
        sf::Event event;
        while (this->window->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                this->window->close();
        }

        this->window->clear();
        for(int i=0; i<=this->size;i++){
            this->window->draw(this->gridMap[i]);
        }
        this->window->display();
    }
}

void Visualiser::setGreen(int loc){
    if(window->isOpen())
    gridMap[loc].setFillColor(sf::Color::Green);
};
void Visualiser::setRed(int loc){
    if(window->isOpen())
    gridMap[loc].setFillColor(sf::Color::Red);
};
void Visualiser::setOrange(int loc){
    if(window->isOpen())
    gridMap[loc].setFillColor(sf::Color(255,153,51));
};
void Visualiser::setWhite(int loc){
    if(window->isOpen())
    gridMap[loc].setFillColor(sf::Color::White);
};
void Visualiser::setBlack(int loc){
    if(window->isOpen())
    gridMap[loc].setFillColor(sf::Color::Black);
};
void Visualiser::setColor(int loc,int r,int g, int b,float opacity){
    if(window->isOpen())
    gridMap[loc].setFillColor(sf::Color(r,g,b, opacity));
};

Visualiser::~Visualiser(){
    delete window;
    delete[] gridMap;
}