#ifndef DATA_TYPES_H_
#define DATA_TYPES_H_

#include <openrave/openrave.h>

namespace benchmarks {

	/**
	 * A bounding box describing bounds for sampling
	 */
	class BoundingBox {
	public:
		/**
		 * Constructor
		 */
	BoundingBox() : xmin(0.0), xmax(0.0), ymin(0.0), ymax(0.0), zmin(0.0), zmax(0.0) {}

		/**
		 * Constructor
		 * @param xmin The minimum x value
		 * @param xmax The maximum x value
		 * @param ymin The minimum y value
		 * @param ymax The maximum y value
		 * @param zmin The minimum z value		 
		 * @param zmax The maximum z value
		 */
		BoundingBox(const double &xmin, 
				   const double &xmax,
				   const double &ymin, 
				   const double &ymax,
				   const double &zmin, 
				   const double &zmax) : xmin(xmin), xmax(xmax), ymin(ymin), ymax(ymax), zmin(zmin), zmax(zmax) {}

		/**
		 * Set the bounds
		 * @param xmin The minimum x value
		 * @param xmax The maximum x value
		 * @param ymin The minimum y value
		 * @param ymax The maximum y value
		 * @param zmin The minimum z value		 
		 * @param zmax The maximum z value
		 */
		void set(const double &x_min, 
				 const double &x_max,
				 const double &y_min, 
				 const double &y_max,
				 const double &z_min, 
				 const double &z_max) {
			xmin = x_min;
			ymax = x_max;
			ymin = y_min;
			ymax = y_max;
			zmin = z_min;
			zmax = z_max;
		}

		
		/**
		 * The minimum x value
		 */
		double xmin;
		
		/**
		 * The maximum x value
		 */
		double xmax;

		/**
		 * The minimum y value
		 */
		double ymin;
		
		/**
		 * The maximum y value
		 */
		double ymax;

		/**
		 * The minimum z value
		 */
		double zmin;
		
		/**
		 * The maximum z value
		 */
		double zmax;
	};

	/**
	 * Type of data for environment collision checking
	 */
	class CollisionData {
	public:
		/**
		 * Constructor
		 */
	CollisionData() : elapsed(0.0), collision(true) {}

		/**
		 * Constructor. 
		 * @param pt The transform being checked for collision
		 * @param elapsed The time taken for the collision check
		 * @param collision True, if the transform is in collision
		 */
	CollisionData(const OpenRAVE::Transform &pt, const double &elapsed, const bool &collision)
		: pt(pt), elapsed(elapsed), collision(collision) {}
		
		/**
		 * The transform being checked for collision
		 */
		OpenRAVE::Transform pt;

		/**
		 * The time taken for the collision check
		 */
		double elapsed;

		/**
		 * True if the transform is in collision
		 */
		bool collision;
	};

	/**
	 * Type of data for self collision checking
	 */
	class SelfCollisionData {
	public:
		/**
		 * Constructor
		 */
	SelfCollisionData() : elapsed(0.0), collision(true) {}

		/**
		 * Constructor
		 * @param pt The dof values checked for self collision
		 * @param elapsed The time taken for the collision check
		 * @param collision True if the transform is in collision
		 */
	SelfCollisionData(const std::vector<double> &pt, const double &elapsed, const bool &collision)
		: pt(pt), elapsed(elapsed), collision(collision) {}

		/**
		 * The dof values checked for self collision
		 */
		std::vector<double> pt;

		/**
		 * The time taken for the collision check
		 */
		double elapsed;

		/**
		 * True if the transform is in collision
		 */
		bool collision;
	};

}

#endif
