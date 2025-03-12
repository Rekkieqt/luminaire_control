#ifndef HELLO_H
#define HELLO_H
//class declaration
class Hello
{
private:
	int id; //data
public:
	//constructor
	Hello( int = 0 );
	//destructor
	~Hello();
	//function
	void run();
};
#endif //HELLO_H 