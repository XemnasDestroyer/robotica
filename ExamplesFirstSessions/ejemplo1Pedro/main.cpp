#include <QtWidgets>
#include "ejemplo1.h"
#include "tests.h"

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    auto start = std::chrono::high_resolution_clock::now();

    ejemplo1 foo;
    foo.show();
    tests();

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
    std::cout << "Tiempo de ejecución: " << duration.count() << " microsegundos" << std::endl;

    return app.exec();
}
