#include "Connection.hpp"
#include "LeapC.h"
#include<iostream>

int main(){
    Connection conn;
    conn.open();
    
    // wait for input
    std::cout << "Press Enter to close the connection..." << std::endl;
    std::cin.get();
    
    conn.close();

    std::cout << "Connection closed." << std::endl;
    std::cin.get();

    // Clean up
    conn.destroy();
    return 0;
}