#include "tests.h"

#include <functional>
#include <thread>
#include <chrono>
#include <ios>
#include <iostream>
#include <qthreadstorage.h>
#include <vector>

tests::tests()
{
    startTest();
}

void tests::startTest()
{
    createAuto(1, 2.3, "hola");
    lambdas();
    threads();
    largeVector();
}

template<typename T1, typename T2, typename T3>
void tests::createAuto(T1 a, T2 b, T3 c)
{
    std::cout << typeid(a).name() << std::endl;
    std::cout << typeid(b).name() << std::endl;
    std::cout << typeid(c).name() << std::endl;
}

void tests::lambdas()
{
    auto saludar = []()
    {
        std::cout << "Saludos" << std::endl;
        return 0;
    };
    saludar();

    auto suma = [](int a, int b)
    {
        std::cout << a + b << std::endl;
    };
    suma(3, 5);

    auto a = 7;
    auto b = 9.3;
    auto suma2 = [a, b]()
    {
        std::cout << a + b << std::endl;
    };
    suma2();
}

void tests::tuple()
{
    std::vector<std::tuple<int, float, std::string>> tuples;
    tuples.push_back(std::make_tuple(1, 2.3, "hola"));
    tuples.push_back(std::make_tuple(4, 3.6, "adios"));
    tuples.push_back(std::make_tuple(14, 100.63, "que se yo"));

    for (auto &tuple : tuples)
    {
        std::cout << std::get<0>(tuple) << " // "
                    << std::get<1>(tuple) << " // "
                    << std::get<2>(tuple) << std::endl;
    }

}

void tests::threads()
{
    //El hilo principal se bloquea hasta que el hilo termine
    std::thread(std::bind(&tests::tuple, this)).join();

    //El hilo va por libre y el hilo principal no espera por el. El hilo creado se destruye cuando termina
    std::thread(std::bind(&tests::tuple, this)).detach();
}

void tests::largeVector()
{
    int elements = (1024*1024*1024)/4;
    std::vector<int> largeVector(elements);

    //lambda
    auto byValue = [](std::vector<int> largeVector){};

    auto start = std::chrono::high_resolution_clock::now();
    byValue(largeVector);
    auto end = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Ejecución por copia: " << totalTime.count() << " ms." << std::endl;

    //lambda
    auto byReference = [](std::vector<int> &largeVector){};

    start = std::chrono::high_resolution_clock::now();
    byReference(largeVector);
    end = std::chrono::high_resolution_clock::now();
    totalTime = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Ejecución por referencia: " << totalTime.count() << " ms." << std::endl;
}