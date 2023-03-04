#include <iostream>
#include "math.h"
#include <istream>

using namespace std;

int sum(int a, int b){
    return a + b;
}


int main() {
    int a,b;
    
    cout <<"inserisci un numero"<<endl;
    cin >> a;
    cout << "inserici un altro numero "<<endl ;
    cin>> b;
    cout<< "somma: "<< sum(a,b);

    return 0;
}


