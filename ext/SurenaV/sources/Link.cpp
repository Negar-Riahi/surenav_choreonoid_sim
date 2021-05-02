#include "headers/Link.h"

 _Link::_Link(short int ID, short int parentID, short int numChilds, short int childID){
     
 }

 short int _Link::getID(){
     return this->ID_;
 }

double _Link::q(){
    return this->q_;
}

short int _Link::getChildID(){
    return this->childID_;
}

short int _Link::getParentID(){
    return this->parentID_;
}