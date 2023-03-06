#include <iostream>



int main(){
    //std::cout<<"hello, world!\a"<<std::endl;
    int ciao = 3;
   // foo = &ciao;
   int ** foo;
   int * fooo;
   int myArray[40];
   myArray[0] = 0;
   myArray[1] = 1;
   int  * fo;
   fo = myArray;
  
   fooo = &ciao;
    foo = &fooo;
    std::cout<<&ciao<<std::endl;
    std::cout<<foo<<std::endl;
    std::cout<<ciao<<std::endl;
    **foo = 10;
    std::cout<<ciao<<std::endl;
    std::cout<<&ciao + 1 <<std::endl;
    std::cout<<fooo<<std::endl;
    std::cout<<*(fo+1)<<std::endl;
    std::cout<<myArray[0]<<std::endl;
    std::cout<<foo<<std::endl;
    std::cout<<**foo+1<<std::endl;
}