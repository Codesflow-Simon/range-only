#include <catch2/catch.hpp>
#include "cameraEmulator.h"

void PolarCoordinate_EQUALS(PolarCoordinate a, PolarCoordinate b) {
    bool result = a.equals(b);
    if (!result) cout << a << " ( " << a.toPoint() << " ) does not equal " << b << " ( " << b.toPoint() << " )\n";
    REQUIRE(result);
}

TEST_CASE() {
    SECTION( "Trivial point test" ) {
        PolarCoordinate a;
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }

    SECTION( "pi/2 point test" ) {
        PolarCoordinate a(2, 0, M_PI/2);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);

        PolarCoordinate c(2, M_PI/2, M_PI/2);
        PolarCoordinate d(a.toPoint());

        PolarCoordinate_EQUALS(c,d);
        PolarCoordinate_EQUALS(a,d);
        PolarCoordinate_EQUALS(b,c);
    }

    SECTION( "pi/2 point test2" ) {
        PolarCoordinate a(2, M_PI/2, 0);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }


    SECTION( "Basic point test" ) {
        PolarCoordinate a(sqrt(12), M_PI/4, 0.6154797);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }
    
    SECTION( "NegativeAltPoint" ) {
        PolarCoordinate a(7, M_PI/6, -M_PI/3);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }

    SECTION( "Beyond pi/2 test" ) {
        PolarCoordinate a(9, 5*M_PI/6, M_PI/3);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }

    SECTION( "Beyond pi/2 and negative" ) {
        PolarCoordinate a(13, 2*M_PI/3, -M_PI/7);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }

    SECTION( "Beyond pi/2 and negative2" ) {
        PolarCoordinate a(1, -1*M_PI/8, M_PI/3);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);    
    }   

    SECTION( "Beyond pi/2 and negative3" ) {
        PolarCoordinate a(4, -5*M_PI/8, -2*M_PI/5);
        PolarCoordinate b(a.toPoint());

        PolarCoordinate_EQUALS(a,b);
    }

    SECTION( "Full circle point test" ) {
    PolarCoordinate a(20, 4*M_PI/3, -2*M_PI/5);
    PolarCoordinate b(a.toPoint());

    PolarCoordinate_EQUALS(a,b);
}
}
