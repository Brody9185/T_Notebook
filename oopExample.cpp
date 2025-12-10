#include <iostream>
#include <string>

// 1. Class Definition
class Dog {
private:
    // 2. Data Members (Attributes) - Encapsulated
    std::string name;
    int age;

public:
    // 3. Constructor
    Dog(std::string dogName, int dogAge) {
        name = dogName;
        age = dogAge;
        std::cout << name << " object has been constructed." << std::endl;
    }

    // 4. Member Functions (Methods)
    void bark() {
        std::cout << name << " barks: Woof! Woof!" << std::endl;
    }

    // 5. Getter Method (Part of Abstraction/Encapsulation)
    // Allows controlled access to a private data member
    int getAge() const {
        return age;
    }

    // 6. Destructor
    ~Dog() {
        std::cout << name << " object has been destroyed." << std::endl;
    }
};

int main() {
    // 7. Object Creation
    // 'myDog' is an object (an instance of the Dog class)
    Dog myDog("Buddy", 5);

    // 8. Accessing a public Member Function
    myDog.bark(); // Calls the bark() method

    // 9. Using a Getter to access private data
    std::cout << myDog.getAge() << " years old." << std::endl;

    // Output:
    // Buddy object has been constructed.
    // Buddy barks: Woof! Woof!
    // 5 years old.
    // Buddy object has been destroyed. (Called automatically at the end of main)

    return 0;
}
