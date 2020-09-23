#include <iostream>
#include <omp.h>

int main()
{
    #pragma omp parallel 
    {
        std::cout << omp_get_thread_num() << "\n";
    }
}