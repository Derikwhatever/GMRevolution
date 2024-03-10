An easy-to-use and convenient and growing library that contains structs and handy scripts for data types such as vectors, quaternions, and various other functions related to their use.

This library can be used to perform rotations that are free of issues such as gimbal lock, an issue typically found with euler-angle based rotations where one of the axis of rotation align with another, resulting in what appears like a loss in a degree of freedom for an objects rotation. Quaternion's solve that issue, and while I recommend doing some reading on how they work, ideally you could work on your 3D projects in GameMaker without having to reinvent the wheel along the way. Adding this library to your project can serve as a strong foundation for getting you started into the realm of making 3D games in GameMaker.

Originally started out as only an adaptation of callmeEthan's Gamemaker_quaternion library that made it compatible and more convenient to use with DragoniteSpam's Vector3 library, but I have since made additions and modified them for use in my own projects. Some of the vector functions are based on or adapted from FoxyOfJungle's TurboGML vector library.

Links to the aforementioned libraries this was originally built on:

https://github.com/callmeEthan/Gamemaker_quaternion

https://github.com/DragoniteSpam-GameMaker-Tutorials/3DCollisions/blob/more-optimizations/scripts/Col_Vector/Col_Vector.gml

https://github.com/FoxyOfJungle/TurboGML/tree/main
