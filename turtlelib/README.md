# Turtlelib Library
A library for handling transformations in SE(2) and other turtlebot-related math.

# Components
- rigid2d - Handles 2D rigid body transformations
- frame_main - Perform some rigid body computations based on user input
- diff_drive - Tracks the robots wheel positions and configuration

# Conceptual Questions
1. We need to be able to ~normalize~ Vector2D objects (i.e., find the unit vector in the direction of a given Vector2D):
   - Propose three different designs for implementing the ~normalize~ functionality

   - Discuss the pros and cons of each proposed method, in light of the C++ Core Guidelines.

   - Which of the methods would you implement and why?

2. What is the difference between a class and a struct in C++?


3. Why is Vector2D a struct and Transform2D a Class (refer to at least 2 specific C++ core guidelines in your answer)?


4. Why are some of the constructors in Transform2D explicit (refer to a specific C++ core guideline in your answer)?


5. Why is Transform2D::inv() declared const while Transform2D::operator*=() is not?
   - Refer to [[https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines#con-constants-and-immutability][C++ Core Guidelines (Constants and Immutability)]] in your answer

# Answers
1. The 3 designs for implementing the ~normalize~ functionality are as follows:
    Defining the function as a helper function of rigid2d library.
        -Pros: The function does not require direct access to members of a class, as a helper function it does not increase coupling (c.4).
        -Cons: Putting it in a class or struct maybe better for comprehension because it would be using related data as input (c.1).
    Defining the function as part of the Vector2D struct.
        -Pros: Putting related data and functions together in a struct is better for comprehension, especially since this function take a vector2D input and returns another Vector2D(c.1).
        -Cons: Increases coupling, since all the members of the struct are public, the function has access to its components anyway(c.4)
    Defining the function as a part of the Transform2D class.
        -Pros:Encapsulation incase the function wants to be a non public member (c.9 and c.8), minimizing exposure of members. 
        -Cons:Unrelated data will be passed into the function since theta angle is not necessary for normalization (C4).
    The main reason I designed normalize to be a helper function in rigid2d instead of a struct or a class is because it does not need direct access to the representation of the class(c.4). This reduces coupling, fewer functions chances of bugs when an object state is modified. It also reduces the the number of functiond that need to be modified after a change in representation.

2. The only difference between a class and a struct in C++ is the default visibility of members. The members of a class are private by default
    while the members of a struct are public by default.

3. Data abstraction, to make it clear that some members are being hidden. When members are non-public classes are preffered to structs
    because of readability (C.8). Vector2D only has public members where anyone can access its x and y components and modify them so they are
    defined as structs. On the otherhand, the object variables of Transform2D need to be private members to avoid accidental modification,
    hence they are defined as a class.
    The use of classes is preferred if the data members have an invariant relationship whereas structs are preferred if the data members are independent to eachother(c.2). In this case the x and y components of the Vector2D do not have an invariant relationship while the Transform2D data members constitute components
    of a transformation matrix which is essentially a rotation matrix and a vector. Therefore the Transform2D has an invariant, making it more suitable as a class.

4. The explicit keyword is used to avoid accidental conversion(for eg implicit type conversions) when calling single argument constructors (c.46).

5. A member function of a class is marked const unless the object's observable state is changed (con.2). In case of Transform2D::inv() it has been marked const 
   because it does not need to modify the object calling it, however Transform2D::operator*=() updates the Transform2D object calling the function, hence it can not be marked as const.


