#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <random>
#include <Eigen/Dense>
#include <SFML/Window.hpp>
#include <SFML/Graphics.hpp>
#include "interpolator.hpp"

float rand_gauss(float sigma = 0.01f){
    static std::minstd_rand gen;
    std::normal_distribution<float> d{0.0f, sigma};
    return d(gen);
}

Eigen::Vector2f test3(){
    Eigen::Vector2f bs1(10.0f, 10.0f);
    Eigen::Vector2f bs2(0.0f, -10.0f);
    Eigen::Vector2f bs3(-10.0f, -10.0f);
    
    Eigen::Vector2f source(0.1f, 1.8f);
    float sol = 1.0f;
    
    float t1 = (bs1 - source).norm() * sol + rand_gauss();
    float t2 = (bs2 - source).norm() * sol + rand_gauss();
    float t3 = (bs3 - source).norm() * sol + rand_gauss();
    
    float R21 = t2-t1;
    float R31 = t3-t1;
    
    Eigen::Vector2f est(0.0f, 0.0f);
    for(int i = 0; i < 5; i++){
        Eigen::Vector2f delta = solver::solve(bs1, bs2, bs3, R21, R31, est);
        est += delta;
        /*std::cout<<"delta:\n"<<delta<<"\n";
        std::cout<<"delta norm:\n"<<delta.norm()<<"\n";
        std::cout<<"est:\n"<<est<<"\n\n";*/
        if(delta.norm() < 1.0e-4f) break;
    }
    return est - source;
}

Eigen::Vector2f test4(){
    Eigen::Vector2f bs1(10.0f, 10.0f);
    Eigen::Vector2f bs2(10.0f, -10.0f);
    Eigen::Vector2f bs3(-10.0f, -10.0f);
    Eigen::Vector2f bs4(-10.0f, 10.0f);
    
    Eigen::Vector2f source(0.1f, 1.8f);
    float sol = 1.0f;
    
    float t1 = (bs1 - source).norm() * sol + rand_gauss();
    float t2 = (bs2 - source).norm() * sol + rand_gauss();
    float t3 = (bs3 - source).norm() * sol + rand_gauss();
    float t4 = (bs4 - source).norm() * sol + rand_gauss();
    
    float R21 = t2-t1;
    float R31 = t3-t1;
    float R41 = t4-t1;
    
    Eigen::Vector2f est(0.0f, 0.0f);
    for(int i = 0; i < 5; i++){
        Eigen::Vector2f delta = solver::solve(bs1, bs2, bs3, bs4, R21, R31, R41, est);
        est += delta;
        /*std::cout<<"delta:\n"<<delta<<"\n";
        std::cout<<"delta norm:\n"<<delta.norm()<<"\n";
        std::cout<<"est:\n"<<est<<"\n\n";*/
        if(delta.norm() < 1.0e-4f) break;
    }
    return est - source;
    
}

struct BS{
    Eigen::Vector2f pos;
    sf::CircleShape sprite;
    BS(Eigen::Vector2f pos) : pos(pos){
        sprite.setRadius(15.0f);
        sprite.setOrigin(15.0f, 15.0f);
        sprite.setPosition(pos[0], pos[1]);
    }
};

Eigen::Vector2f simulate_measurement(Eigen::Vector2f source_pos,
                                     Eigen::Vector2f bs1, Eigen::Vector2f bs2,
                                     Eigen::Vector2f bs3, Eigen::Vector2f bs4,
                                     float sigma){
    
    float t1 = (bs1 - source_pos).norm() + rand_gauss(sigma);
    float t2 = (bs2 - source_pos).norm() + rand_gauss(sigma);
    float t3 = (bs3 - source_pos).norm() + rand_gauss(sigma);
    float t4 = (bs4 - source_pos).norm() + rand_gauss(sigma);
    
    float R21 = t2-t1;
    float R31 = t3-t1;
    float R41 = t4-t1;
    Eigen::Vector2f est(150.0f, 150.0f);
    for(int i = 0; i < 8; i++){
        Eigen::Vector2f delta = solver::solve(bs1, bs2, bs3, bs4, R21, R31, R41, est);
        est += delta;
        if(delta.norm() < 1.0e-4f) break;
    }
    return est;
}

Eigen::Vector2f simulate_measurement(Eigen::Vector2f source_pos,
                                     std::vector<BS>& base_stations,
                                     float sigma){
    
    Eigen::Vector2f est(500.0f, 500.0f);
    
    std::vector<Eigen::Vector2f> bs;
    bs.reserve(base_stations.size());
    for(auto& i : base_stations){
        bs.push_back(i.pos);
    }
    
    std::vector<float> RRs;
    RRs.reserve(base_stations.size() - 1);
    
    float t0 = (bs[0] - source_pos).norm();
    
    for(size_t i = 0; i < base_stations.size()-1; i++){
        RRs.push_back( ((bs[i+1] - source_pos).norm() - t0) + rand_gauss(sigma) );
    }
    
    for(int i = 0; i < 8; i++){
        Eigen::Vector2f delta = solver::solve(bs, RRs, est);
        est += delta;
        if(delta.norm() < 1.0e-4f) break;
    }
    return est;
}

std::vector<BS> generate_stations(int num, float sigma = 0.0f){
    std::vector<BS> result;
    result.reserve(num);
    float x_cent = 500.0f;
    float y_cent = 500.0f;
    float radius = 300.0f;
    float delta_phi = 2.0f * 3.1415f / num;
    float phi = 0.0f;
    for(int i = 0; i < num; i++){
        float x = (rand_gauss(sigma) + radius) * std::cos(phi) + x_cent;
        float y = (rand_gauss(sigma) + radius) * std::sin(phi) + y_cent;
        Eigen::Vector2f pos(x,y);
        BS b(pos);
        result.push_back(b);
        phi += delta_phi;
    }
    return result;
}

int main(int argc, char** argv){
	sf::RenderWindow window(sf::VideoMode(1024, 1024), "SFML window");
    window.setFramerateLimit(60);
    
    int no_stations = 4;
    float base_sigma = 0.0f;
    if(argc >= 2){
        no_stations = std::stoi(argv[1]);
    }
    if(argc == 3){
        base_sigma = std::stof(argv[2]);
    }
    
    auto base_stations = generate_stations(no_stations, base_sigma);
    
    sf::CircleShape source;
    source.setPosition(300.0f, 300.0f);
    source.setRadius(5.0f);
    source.setOrigin(5.0f, 5.0f);
    source.setFillColor(sf::Color::Blue);
    
    std::vector<sf::CircleShape> estim;
    for(size_t i = 0; i < 1000; i++){
        sf::CircleShape e;
        e.setPosition(20.0f, 20.0f);
        e.setRadius(1.0f);
        e.setOrigin(1.0f, 1.0f);
        e.setFillColor(sf::Color::Red);
        estim.push_back(e);
    }
    
    float sigma = 1.0f;
    bool always_simulate = false;
	while (window.isOpen()){
        sf::Event event;
        while (window.pollEvent(event)){
            if (event.type == sf::Event::Closed){
                window.close();
			}
			if (event.type == sf::Event::KeyPressed){
                if(event.key.code == sf::Keyboard::C){
                    always_simulate = !always_simulate;
                }
                if (event.key.code == sf::Keyboard::Up){
                    sigma*=1.1f;
                    std::cout<<sigma<<std::endl;
                }
                if (event.key.code == sf::Keyboard::Down){
                    sigma*=0.9f;
                    std::cout<<sigma<<std::endl;
                }
				if (event.key.code == sf::Keyboard::Space){
                    Eigen::Vector2f source_pos(source.getPosition().x, source.getPosition().y);
                    
                    /*float t_average = (bs1.pos - source_pos).norm()
                                    + (bs2.pos - source_pos).norm()
                                    + (bs3.pos - source_pos).norm()
                                    + (bs4.pos - source_pos).norm();
                    t_average /= 4.0f;
                    std::cout<<"average time: "<<t_average<<std::endl;*/
                    float avrg_error = 0.0f;
                    for(auto& e : estim){
                        Eigen::Vector2f res = simulate_measurement(source_pos, base_stations, sigma);
                        e.setPosition(res[0], res[1]);
                        avrg_error += (res - source_pos).squaredNorm();
                    }
                    avrg_error /= float(estim.size());
                    std::cout<<"average error: "<<std::sqrt(avrg_error)<<std::endl;
				}
			}
        }
        if(sf::Mouse::isButtonPressed(sf::Mouse::Left)){
            auto p = sf::Mouse::getPosition(window);
            source.setPosition(p.x, p.y);
        }
        if(always_simulate){
            Eigen::Vector2f source_pos(source.getPosition().x, source.getPosition().y);
            for(auto& e : estim){
                Eigen::Vector2f res = simulate_measurement(source_pos, base_stations, sigma);
                e.setPosition(res[0], res[1]);
            }
        }
        
        
		window.clear();
        for(auto& i : base_stations){
            window.draw(i.sprite);
        }
        window.draw(source);
        for(auto& e : estim){
            window.draw(e);
        }
        window.display();
	}
	
}









