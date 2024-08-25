#include<cmath>
#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Dense>
#include<iostream>
#include<eigen3/Eigen/Geometry>

Eigen::Vector3f rotateVector(Eigen::Vector3f vector, float angle) {
    float angleToPi = angle / 180.0f * acos(-1);
    Eigen::Matrix3f rotateMatrix;
    rotateMatrix << cos(angleToPi), -sin(angleToPi), 0.0f, sin(angleToPi), cos(angleToPi), 0.0f, 0, 0, 1;
    Eigen::Vector3f result = rotateMatrix * vector;
    return result;
}

Eigen::Matrix3Xf transitionVector(Eigen::Matrix3Xf homogenousVector, Eigen::Vector3f transitionVector) {
    Eigen::Matrix3f transitionMatrix;
    transitionMatrix << 1, 0, transitionVector[0],
                        0, 1, transitionVector[1],
                        0, 0, transitionVector[2];
    return transitionMatrix * homogenousVector;
}

class Object
{
public:
    Object() { std::cout << "object constructor" << std::endl; }
};

struct BVHSAHBucket {
    // Bounds3 bounds;
    int primCount;
    std::vector<Object*> primitives;

    BVHSAHBucket() {
        // bounds = Bounds3();
        // primitives = std::vector<Object*>();
        std::cout << "BVHSAHBucket contructor" << std::endl;
    }
};

int main(){

    // Basic Example of cpp
    std::array<BVHSAHBucket, 3> buckets;
    std::vector<Object*> objects;
    for (int i = 0; i < 30; i++)
    {
        objects.push_back(new Object());
    }

    for (int i = 0; i < 3; i++)
    {
        buckets[i].primitives.push_back(objects[i]);
    }
    
    for (int i = 0; i < 3; i++)
    {
        std::cout << "primitive size: " << buckets[i].primitives.size() << std::endl;
    }
    return 0;
}

void testEigenFunc()
{
    std::cout << "Example of cpp \n";
    float a = 1.0, b = 2.0;
    std::cout << a << std::endl;
    std::cout << a/b << std::endl;
    std::cout << std::sqrt(b) << std::endl;
    std::cout << std::acos(-1) << std::endl;
    std::cout << std::sin(30.0/180.0*acos(-1)) << std::endl;

    // Example of vector
    std::cout << "Example of vector \n";
    // vector definition
    Eigen::Vector3f v(1.0f,2.0f,3.0f);
    Eigen::Vector3f w(1.0f,0.0f,0.0f);
    // vector output
    std::cout << "Example of output \n";
    std::cout << v << std::endl;
    // vector add
    std::cout << "Example of add \n";
    std::cout << v + w << std::endl;
    // vector scalar multiply
    std::cout << "Example of scalar multiply \n";
    std::cout << v * 3.0f << std::endl;
    std::cout << 2.0f * v << std::endl;
    std::cout << "Example of dot product \n";
    std::cout << v.dot(w) << std::endl;

    // Example of matrix
    std::cout << "Example of matrix \n";
    // matrix definition
    Eigen::Matrix3f i,j;
    i << 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0;
    j << 2.0, 3.0, 1.0, 4.0, 6.0, 5.0, 9.0, 7.0, 8.0;
    // matrix output
    std::cout << "Example of output \n";
    std::cout << i << std::endl;
    std::cout << "matrix add i + j \n";
    std::cout << i + j << std::endl;
    std::cout << "matrix scalar multiply i * 2.0 \n";
    std::cout << i * 2 << std::endl;
    std::cout << "matrix multiply i * j \n";
    std::cout << i * j << std::endl;
    std::cout << "matrix multiply vector i * v \n";
    std::cout << i * v << std::endl;

    Eigen::Vector2f vv(2.0f, 1.0f);
    std::cout << "vv homogeneous" << std::endl;
    std::cout << vv.homogeneous() << std::endl;
    Eigen::Vector3f rotateRes = rotateVector(vv.homogeneous(), 45.0f);
    std::cout << "rotate vector -45" << std::endl;
    std::cout << rotateRes << std::endl;
    Eigen::Matrix3Xf homogengousVector(3,2);
    homogengousVector << 0, rotateRes[0], 0, rotateRes[1], 1, 1;
    std::cout << "homogengousVector" << std::endl;
    std::cout << homogengousVector << std::endl;
    std::cout << "start transition" << std::endl;
    Eigen::Vector2f transition(1, 2);
    Eigen::Matrix3Xf transitionResult = transitionVector(homogengousVector, transition.homogeneous());
    std::cout << transitionResult << std::endl;
    std::cout << "multiply vector by it's transpose" << std::endl;
    Eigen::Vector3f vectorDot(2, 3, 1);
    Eigen::RowVector3f vectorDotT = vectorDot.transpose();
    Eigen::Matrix3f vectorDotResult = vectorDot * vectorDotT;
    std::cout << vectorDotResult << std::endl;
    std::cout << vectorDot.z() << std::endl;
}
