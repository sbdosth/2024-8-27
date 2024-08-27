#include <iostream>

using namespace std;

int main(){
    int a;

    cout<<"请输入1~10以内数字："<<endl;

    cin>>a;

    if(a<1||a>10){
         cout<<"数字超过范围，请重试"<<endl;
    }
       

    else
        for(int i=0;i<a;i++){
            cout << "Hello,RoboMaster!"<<endl;
        }

    return 0;
}