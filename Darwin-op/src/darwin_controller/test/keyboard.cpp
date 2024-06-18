#include <iostream>
#include <conio.h>
#include <thread>

class keyboard
{
    std::thread t1, t2, t3;
    public:
    void initial()
    {
        t1 = std::thread(&keyboard::watch_board, this);
        t2 = std::thread(&keyboard::th1, this);
        t3 = std::thread(&keyboard::th2, this);
    }
    
    void thread_join()
    {
        t1.join();
        t2.join();
        t3.join();
    }
    void watch_board()
    {
        int ch;
        while (1)
        {
            if (kbhit())
            {
                ch = getch();
                if (ch == 27)
                {
                    break;
                }
            }
            else
            {
                ch = 0;
            }
            usleep(5000);
            std::cout<<"key is "<<ch<<std::endl;
            // LOG(INFO)<<"key is "<<ch;
            // lock_guard<mutex> lockGuard(m);
            // date.Key = ch;
            // date.Key = 49;
        }
    }

    void th1()
    {
        while (1)
        {
            std::cout<<"in th1"<<std::endl;
            usleep(100000);
        }
        
    }

    void th2()
    {
        while (1)
        {
            std::cout<<"in th2"<<std::endl;
            usleep(100000);
        }
    }
};

// void watch_board()
// {
//     int ch;
//     while (1)
//     {
//         if (kbhit())
//         {
//             ch = getch();
//             if (ch == 27)
//             {
//                 break;
//             }
//         }
//         else
//         {
//             ch = 0;
//         }
//         usleep(5000);
//         std::cout<<"key is "<<ch<<std::endl;
//         // LOG(INFO)<<"key is "<<ch;
//         // lock_guard<mutex> lockGuard(m);
//         // date.Key = ch;
//         // date.Key = 49;
//     }
// }
// void th1()
// {
//     while (1)
//     {
//         std::cout<<"in th1"<<std::endl;
//         usleep(100000);
//     }
// }
// void th2()
// {
//     while (1)
//     {
//         std::cout<<"in th2"<<std::endl;
//         usleep(100000);
//     }
// }

int main(int argc, char** argv)
{
    // int ch;
    // while (1)
    // {
    //     if (kbhit())
    //     {
    //         ch = getch();
    //         if (ch == 27)
    //         {
    //             break;
    //         }
    //     }
    //     else
    //     {
    //         ch = 0;
    //     }
    //     usleep(5000);
    //     std::cout<<"key is "<<ch<<std::endl;
    //     // LOG(INFO)<<"key is "<<ch;
    //     // lock_guard<mutex> lockGuard(m);
    //     // date.Key = ch;
    //     // date.Key = 49;
    // }
    // std::thread t1(&keyboard::watch_board);
    // std::thread t2(th1);
    // std::thread t3(th2);
    // t1.join();
    // t2.join();
    // t3.join();
    keyboard k;
    k.initial();
    k.thread_join();
    std::cout << "main thread end!" << std::endl;
    // sleep(2);
    return 0;
}