//
// Created by zft on 19-12-15.
//

#ifndef OFFBOARD_MULTI_TEST_H
#define OFFBOARD_MULTI_TEST_H

class offboard{
public:
    offboard(){
    };
    static offboard* getInstance();
    void Oninit();
private:
   offboard* multi_init;
};



#endif //OFFBOARD_MULTI_TEST_H
