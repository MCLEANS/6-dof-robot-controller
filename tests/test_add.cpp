#include "catch2/catch.hpp"

int add(int a, int b){
    return a+b;
};





TEST_CASE("ADDITION","[ADD]"){
    REQUIRE(add(2,3) == 5);
}