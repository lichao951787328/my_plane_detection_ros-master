/*
 * @description: 
 * @param : 
 * @return: 
 */
#include <iostream>
#include <set>
using namespace std;
int main(int argc, char** argv)
{
  set<int> s;
  s.insert(3);
  s.insert(5);
  s.insert(2);
  s.insert(1);
  cout<<*(s.end())<<endl;
  cout<<*(s.begin())<<endl;
  return 0;
}