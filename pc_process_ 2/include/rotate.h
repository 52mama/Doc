#include <vector>
#include <iostream>
#include <cmath>
using namespace std;

class Point3
{
    friend std::ostream& operator<<(std::ostream& os,const Point3& p);
    
    public:
    Point3() 
    : already_set (false)
    { 
        x = 0; 
        y = 0; 
        z = 0; 
        already_set = false;
    }
    Point3(const Point3& other)
    {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
        already_set = true;
    }
    Point3(double x,double y,double z)
    {
        this->x = x;
        this->y = y;
        this->z = z;
        already_set = true;
    }
    Point3 operator-(const Point3& p) const
    {
        return Point3( this->x-p.x , this->y-p.y , this->z-p.z);
    }
    Point3 operator+(const Point3& p) const
    {
        return Point3(this->x+p.x,this->y+p.y,this->z+p.z);
    }
    Point3 operator=(const Point3& p) const
    {
        return Point3( p.x , p.y , p.z ) ;
    }
    
    protected:
    double x,y,z;
    bool already_set;
};

std::ostream& operator<<(std::ostream& os,const Point3& p)
{
    os<< p.x <<" "<< p.y <<" "<< p.z <<endl;
    return os;
}

class Vector3 : public Point3
{
    public:
    Vector3() : Point3() { ; }
    Vector3(const Vector3& v) : Point3(v) { ; }
    Vector3(double x,double y,double z) : Point3(x,y,z) { ; }
    
    static double dot(const Vector3& v1,const Vector3& v2)
    {
        return v1.dot(v2);
    }
    double dot(const Vector3& v) const 
    {
        return this->x * v.x + this->y * v.y + this->z * v.z ;
    }

    static Vector3 cross(const Vector3& v1,const Vector3& v2)   
    {
        return v1.cross(v2);
    } 
    Vector3 cross(const Vector3& v) const
    {
        int x = this->x;
        int y = this->y;
        int z = this->z;
        return Vector3( y*v.z - z*v.y , z*v.x - x*v.z , x*v.y - y*v.x ) ;
    }
    
    static double norm(const Vector3& v)
    {
        return v.norm();
    }
    double norm() const
    {
        return sqrt( this->dot( *this ) );
    }
    
}; 


class Rotate_axis
{
    public:
    // construct
    Rotate_axis()   { ; }
    Rotate_axis(Vector3 rotate_axis , Point3 rotate_point )
    {
        already_set_point = true;
        already_set_vector = true;
        this->rotate_axis = rotate_axis;
        this->rotate_point = rotate_point;
    }
    
    // setter
    void setRotateAxis(Vector3 rotate_axis)
    {
        this->rotate_axis = rotate_axis;
    }
    void setRotatePoint(Point3 rotate_point)
    {
        this->rotate_point = rotate_point;
    }

    // process
    std::vector<Point3> rotate(std::vector<Point3> points)
    {   
        

        std::vector<Point3> res;
        for(auto i : points)
        {
            res.push_back( rotate(i) );
        }
        return res;
    }
    private:

    Vector3 rotate_axis;
    Point3 rotate_point;
    Point3 rotate(Point3 point)
    {
        return Point3(0,0,0);
    }
};



class Rotate_plane
{
    public:
    Rotate_plane(std::vector<double> plane1,std::vector<double> plane2)
    {
        ;
    }

    private:
    Rotate_axis rotate_axis;
};