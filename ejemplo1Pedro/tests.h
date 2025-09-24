#ifndef tests_h
#define tests_h

class tests
{
public:
    tests();

    void startTest();

    template<typename T1, typename T2, typename T3>
    void createAuto(T1 a, T2 b, T3 c);

    void lambdas();
    void tuple();
    void threads();
    void largeVector();
};

#endif // tests_h
