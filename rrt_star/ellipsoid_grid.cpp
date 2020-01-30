#include "ellipsoid_grid.h"


namespace kamaz {
namespace hagen {

double*  Ellipsoid::ellipsoid_grid ( int n, int ng ){
  double h;
  int ii;
  int i;
  int j;
  int k;
  int m;
  int ng2;
  int ni;
  int nj;
  int nk;
  int np;
  double p[3*8];
  double rmin;
  double x;
  double *xyz;
  double y;
  double z;
  ng2 = 0;
  xyz = new double[3*ng];
  rmin = r8vec_min ( 3, r );
  if ( r[0] == rmin )
  {
    h = 2.0 * r[0] / ( double ) ( 2 * n + 1 );
    ni = n;
    nj = i4_ceiling ( r[1] / r[0] ) * ( double ) ( n );
    nk = i4_ceiling ( r[2] / r[0] ) * ( double ) ( n );
  }
  else if ( r[1] == rmin )
  {
    h = 2.0 * r[1] / ( double ) ( 2 * n + 1 );
    nj = n;
    ni = i4_ceiling ( r[0] / r[1] ) * ( double ) ( n );
    nk = i4_ceiling ( r[2] / r[1] ) * ( double ) ( n );
  }
  else
  {
    h = 2.0 * r[2] / ( double ) ( 2 * n + 1 );
    nk = n;
    ni = i4_ceiling ( r[0] / r[2] ) * ( double ) ( n );
    nj = i4_ceiling ( r[1] / r[2] ) * ( double ) ( n );
  }

  for ( k = 0; k <= nk; k++ )
  {
    z = c[2] + ( double ) ( k ) * h;
    for ( j = 0; j <= nj; j++ )
    {
      y = c[1] + ( double ) ( j ) * h;
      for ( i = 0; i <= ni; i++ )
      {
        x = c[0] + ( double ) ( i ) * h;
        if ( 1.0 < pow ( ( x - c[0] ) / r[0], 2 )
                 + pow ( ( y - c[1] ) / r[1], 2 )
                 + pow ( ( z - c[2] ) / r[2], 2 ) )
        {
          break;
        }
        np = 0;
        p[0+np*3] = x;
        p[1+np*3] = y;
        p[2+np*3] = z;
        np = 1;

        if ( 0 < i )
        {
          for ( m = 0; m < np; m++ )
          {
            p[0+(np+m)*3] = 2.0 * c[0] - p[0+m*3];
            p[1+(np+m)*3] = p[1+m*3];
            p[2+(np+m)*3] = p[2+m*3];
          }
          np = 2 * np;
        }

        if ( 0 < j )
        {
          for ( m = 0; m < np; m++ )
          {
            p[0+(np+m)*3] = p[0+m*3];
            p[1+(np+m)*3] = 2.0 * c[1] - p[1+m*3];
            p[2+(np+m)*3] = p[2+m*3];
          }
          np = 2 * np;
        }

        if ( 0 < k )
        {
          for ( m = 0; m < np; m++ )
          {
            p[0+(np+m)*3] = p[0+m*3];
            p[1+(np+m)*3] = p[1+m*3];
            p[2+(np+m)*3] = 2.0 * c[2] - p[2+m*3];
          }
          np = 2 * np;
        }
   
        for ( m = 0; m < np; m++ )
        {
          for ( ii = 0; ii < 3; ii++ )
          {
            xyz[ii+(ng2+m)*3] = p[ii+m*3];
          }
        }
        ng2 = ng2 + np;
      }
    }
  }
  return xyz;
}

int Ellipsoid::ellipsoid_grid_count( int n, Eigen::Vector3d radios, Eigen::Vector3d center){
  
  r[0] = radios[0];
  r[1] = radios[1];
  r[2] = radios[2];
  c[0] = center[0];
  c[1] = center[1];
  c[2] = center[2];
  double h;
  int i;
  int j;
  int k;
  int m;
  int ng;
  int ni;
  int nj;
  int nk;
  int np;
  double rmin;
  double x;
  double y;
  double z;

  ng = 0;

  rmin = r8vec_min ( 3, r );

  if ( r[0] == rmin )
  {
    h = 2.0 * r[0] / ( double ) ( 2 * n + 1 );
    ni = n;
    nj = i4_ceiling ( r[1] / r[0] ) * ( double ) ( n );
    nk = i4_ceiling ( r[2] / r[0] ) * ( double ) ( n );
  }
  else if ( r[1] == rmin )
  {
    h = 2.0 * r[1] / ( double ) ( 2 * n + 1 );
    nj = n;
    ni = i4_ceiling ( r[0] / r[1] ) * ( double ) ( n );
    nk = i4_ceiling ( r[2] / r[1] ) * ( double ) ( n );
  }
  else
  {
    h = 2.0 * r[2] / ( double ) ( 2 * n + 1 );
    nk = n;
    ni = i4_ceiling ( r[0] / r[2] ) * ( double ) ( n );
    nj = i4_ceiling ( r[1] / r[2] ) * ( double ) ( n );
  }

  for ( k = 0; k <= nk; k++ )
  {
    z = c[2] + ( double ) ( k ) * h;
    for ( j = 0; j <= nj; j++ )
    {
      y = c[1] + ( double ) ( j ) * h;
      for ( i = 0; i <= ni; i++ )
      {
        x = c[0] + ( double ) ( i ) * h;
        if ( 1.0 < pow ( ( x - c[0] ) / r[0], 2 )
                 + pow ( ( y - c[1] ) / r[1], 2 )
                 + pow ( ( z - c[2] ) / r[2], 2 ) )
        {
          break;
        }

        np = 1;
        if ( 0 < i )
        {
          np = 2 * np;
        }
        if ( 0 < j )
        {
          np = 2 * np;
        }
        if ( 0 < k )
        {
          np = 2 * np;
        }
        ng = ng + np;
      }
    }
  }
  return ng;
}

int Ellipsoid::i4_ceiling( double x )
{
  int value;

  value = ( int ) x;

  if ( value < x )
  {
    value = value + 1;
  }

  return value;
}

void Ellipsoid::r83vec_print_part( int n, double a[], Eigen::Vector3d center_pose, Eigen::Matrix3d rotation_matrix, std::string file_name )
{
    int i;
    if ( n <= 0 )
    {
        return;
    }
    std::vector<double> edges; 
    int count = 0;
    for ( i = 0; i < n - 2; i++ )
    {
        Eigen::Vector3d point;
        point<< a[0+i*3], a[1+i*3], a[2+i*3];
        Eigen::Vector3d ff = point.transpose()*rotation_matrix;
        std::cout<< center_pose << std::endl;
        std::cout<< ff << std::endl;
        ff = ff + center_pose;
        std::cout<< ff << std::endl;
        edges.push_back(ff[0]);
        edges.push_back(ff[1]);
        edges.push_back(ff[2]);
        count +=1;
    }
    cnpy::npy_save(file_name, &edges[0],{(unsigned int)1, (unsigned int)count, (unsigned int)3},"w");
  return;
}

void Ellipsoid::generate_points( int n, Eigen::Vector3d radios, Eigen::Vector3d center_pose
        , Eigen::Matrix3d rotation_matrix, std::shared_ptr<Eigen::MatrixXd>& random_points_tank)
{

    Eigen::Vector3d c(0,0,0);
    int ng = ellipsoid_grid_count(n, radios, c);
    std::cout << "\n";
    std::cout << "  Number of grid points will be " << ng << "\n";
    double* a = ellipsoid_grid(n, ng);
    *random_points_tank = Eigen::MatrixXd::Zero(ng-2, 3);
    int i;
    if ( n <= 0 )
    {
        return;
    }
    
    int count = 0;
    for ( i = 0; i < ng - 2; i++ )
    {
        Eigen::Vector3d point;
        point<< a[0+i*3], a[1+i*3], a[2+i*3];
        Eigen::Vector3d ff = point.transpose()*rotation_matrix;
        ff = ff + center_pose;
        (*random_points_tank).row(i) = ff;
        count +=1;
    }
  return;
}

double Ellipsoid::r8vec_min( int n, double r8vec[] ){
  int i;
  double value;

  value = r8vec[0];

  for ( i = 1; i < n; i++ )
  {
    if ( r8vec[i] < value )
    {
      value = r8vec[i];
    }
  }
  return value;
}

void Ellipsoid::timestamp ( ){
  # define TIME_SIZE 40 
  static char time_buffer[TIME_SIZE];
  const struct std::tm *tm_ptr;
  size_t len;
  std::time_t now;
  now = std::time ( NULL );
  tm_ptr = std::localtime ( &now );
  len = std::strftime ( time_buffer, TIME_SIZE, "%d %B %Y %I:%M:%S %p", tm_ptr );
  std::cout << time_buffer << "\n";
  return;
  # undef TIME_SIZE
}

}
}