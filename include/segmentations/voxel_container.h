#ifndef _Voxel_Container_H_Tony_2016_Dec_7_
#define _Voxel_Container_H_Tony_2016_Dec_7_


#include <string.h>
#include <vector>
#include <cstddef>

#include <pcl/pcl_macros.h>

namespace pcl
{
	namespace UrbanRec
	{
		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief @b Octree container class that can serve as a base to construct own leaf node container classes.
		*  \author Julius Kammerl (julius@kammerl.de)
		*/
		enum VOXEL_OCCUPYFLAG
		{
			null_voxel = 0,
			occupied_voxel = 1,
			inner_voxel = 2
		};

		struct  VoxelFeature
		{//voxel features
			Eigen::Vector3f vRefPos; //voxel centre pos
			int		ptsNum;

			float	fProjDens;  //水平投影点密度
			
			/** \brief Stores the mean value (center of mass) of the cloud */
			Eigen::Vector3f centroid;  //点云的重心 

			/** \brief Major eigen vector */
			Eigen::Vector3f major_axis_;

			/** \brief Middle eigen vector */
			Eigen::Vector3f middle_axis_;

			/** \brief Minor eigen vector */
			Eigen::Vector3f minor_axis_;

			/** \brief Major eigen value */
			float major_value_;

			/** \brief Middle eigen value */
			float middle_value_;

			/** \brief Minor eigen value */
			float minor_value_;
		};

		class VoxelContainerBase
		{
		public:
			int		label;     //-1 unlabeled
			int  occupyFlag;  //0: null, 1: occupied, 2: inner

		public:
			/** \brief Empty constructor. */
			VoxelContainerBase ()
			{
			}

			/** \brief Empty constructor. */
			VoxelContainerBase (const VoxelContainerBase&)
			{
			}

			/** \brief Empty deconstructor. */
			virtual
				~VoxelContainerBase ()
			{
			}

			/** \brief Equal comparison operator
			*/
			virtual bool
				operator== (const VoxelContainerBase&) const
			{
				return false;
			}

			/** \brief Inequal comparison operator
			* \param[in] other VoxelContainerBase to compare with
			*/
			bool
				operator!= (const VoxelContainerBase& other) const
			{
				return (!operator== (other));
			}

			/** \brief Pure abstract method to get size of container (number of indices)
			* \return number of points/indices stored in leaf node container.
			*/
			virtual size_t
				getSize () const
			{
				return 0u;
			}

			/** \brief Pure abstract reset leaf node implementation. */
			virtual void
				reset () = 0;

			/** \brief Empty addPointIndex implementation. This leaf node does not store any point indices.
			*/
			void
				addPointIndex (const int&)
			{
			}

			/** \brief Empty getPointIndex implementation as this leaf node does not store any point indices.
			*/
			void
				getPointIndex (int&) const
			{
			}

			/** \brief Empty getPointIndices implementation as this leaf node does not store any data. \
			*/
// 			void
// 				getPointIndices (std::vector<int>&) const
// 			{
// 			}

			std::vector<int>*
				getPointIndices () const
			{
				//std::vector<int> p;
				return 0;
			}
		};

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief @b Octree container class that does not store any information.
		* \note Can be used for occupancy trees that are used for checking only the existence of leaf nodes in the tree
		* \author Julius Kammerl (julius@kammerl.de)
		*/

		class VoxelContainerEmpty : public VoxelContainerBase
		{
		public:
			/** \brief Empty constructor. */
			VoxelContainerEmpty () :
			  VoxelContainerBase ()
			  {
			  }

			  /** \brief Empty constructor. */
			  VoxelContainerEmpty (const VoxelContainerEmpty&) :
			  VoxelContainerBase ()
			  {
			  }

			  /** \brief Empty deconstructor. */
			  virtual
				  ~VoxelContainerEmpty ()
			  {
			  }

			  /** \brief Octree deep copy method */
			  virtual VoxelContainerEmpty *
				  deepCopy () const
			  {
				  return (new VoxelContainerEmpty (*this));
			  }

			  /** \brief Abstract get size of container (number of DataT objects)
			  * \return number of DataT elements in leaf node container.
			  */
			  virtual size_t
				  getSize () const
			  {
				  return 0;
			  }

			  /** \brief Abstract reset leaf node implementation. */
			  virtual void
				  reset ()
			  {

			  }

			  /** \brief Empty addPointIndex implementation. This leaf node does not store any point indices.
			  */
			  void
				  addPointIndex (int)
			  {
			  }

			  /** \brief Empty getPointIndex implementation as this leaf node does not store any point indices.
			  */
			  int
				  getPointIndex () const
			  {
				  assert("getPointIndex: undefined point index");
				  return -1;
			  }

			  /** \brief Empty getPointIndices implementation as this leaf node does not store any data. \
			  */
// 			  void
// 				  getPointIndices (std::vector<int>&) const
// 			  {
// 			  }

			  std::vector<int>*
				  getPointIndices () const
			  {
				  return 0;
			  }
		};

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief @b Octree container class that does store a single point index.
		* \note Enables the octree to store a single DataT element within its leaf nodes.
		* \author Julius Kammerl (julius@kammerl.de)
		*/
		class VoxelContainerPointIndex : public VoxelContainerBase
		{
		public:
			/** \brief Empty constructor. */
			VoxelContainerPointIndex () :
			  VoxelContainerBase (), data_ ()
			  {
				  reset ();
			  }

			  /** \brief Empty constructor. */
			  VoxelContainerPointIndex (const VoxelContainerPointIndex& source) :
			  VoxelContainerBase (), data_ (source.data_)
			  {
			  }

			  /** \brief Empty deconstructor. */
			  virtual
				  ~VoxelContainerPointIndex ()
			  {
			  }

			  /** \brief Octree deep copy method */
			  virtual VoxelContainerPointIndex*
				  deepCopy () const
			  {
				  return (new VoxelContainerPointIndex (*this));
			  }

			  /** \brief Equal comparison operator
			  * \param[in] other VoxelContainerBase to compare with
			  */
			  virtual bool
				  operator== (const VoxelContainerBase& other) const
			  {
				  const VoxelContainerPointIndex* otherConDataT = dynamic_cast<const VoxelContainerPointIndex*> (&other);

				  return (this->data_ == otherConDataT->data_);
			  }

			  /** \brief Add point index to container memory. This container stores a only a single point index.
			  * \param[in] data_arg index to be stored within leaf node.
			  */
			  void
				  addPointIndex (int data_arg)
			  {
				  data_ = data_arg;
			  }

			  /** \brief Retrieve point index from container. This container stores a only a single point index
			  * \return index stored within container.
			  */
			  int
				  getPointIndex () const
			  {
				  return data_;
			  }

			  /** \brief Retrieve point indices from container. This container stores only a single point index
			  * \param[out] data_vector_arg vector of point indices to be stored within data vector
			  */
// 			  void
// 				  getPointIndices (std::vector<int>& data_vector_arg) const
// 			  {
// 				  if (data_>=0)
// 					  data_vector_arg.push_back (data_);
// 			  }

			  std::vector<int>*
				  getPointIndices () const
			  {
				  //std::vector<int> p;
				  return 0;
			  }

			  /** \brief Get size of container (number of DataT objects)
			  * \return number of DataT elements in leaf node container.
			  */
			  size_t
				  getSize () const
			  {
				  return data_<0 ? 0 : 1;
			  }

			  /** \brief Reset leaf node memory to zero. */
			  virtual void
				  reset ()
			  {
				  data_ = -1;
			  }
		protected:
			/** \brief Point index stored in octree. */
			int data_;
		};

		//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		/** \brief @b Octree container class that does store a vector of point indices.
		* \note Enables the octree to store multiple DataT elements within its leaf nodes.
		* \author Julius Kammerl (julius@kammerl.de)
		*/
		class VoxelContainerPointIndices : public VoxelContainerBase
		{
		public:
			/** \brief Empty constructor. */
			VoxelContainerPointIndices () :
			  VoxelContainerBase (), leafDataTVector_ ()
			  {
			  }

			  /** \brief Empty constructor. */
			  VoxelContainerPointIndices (const VoxelContainerPointIndices& source) :
			  VoxelContainerBase (), leafDataTVector_ (source.leafDataTVector_)
			  {
			  }

			  /** \brief Empty deconstructor. */
			  virtual
				  ~VoxelContainerPointIndices ()
			  {
			  }

			  /** \brief Octree deep copy method */
			  virtual VoxelContainerPointIndices *
				  deepCopy () const
			  {
				  return (new VoxelContainerPointIndices (*this));
			  }

			  /** \brief Equal comparison operator
			  * \param[in] other VoxelContainerDataTVector to compare with
			  */
			  virtual bool
				  operator== (const VoxelContainerBase& other) const
			  {
				  const VoxelContainerPointIndices* otherConDataTVec = dynamic_cast<const VoxelContainerPointIndices*> (&other);

				  return (this->leafDataTVector_ == otherConDataTVec->leafDataTVector_);
			  }

			  /** \brief Add point index to container memory. This container stores a vector of point indices.
			  * \param[in] data_arg index to be stored within leaf node.
			  */
			  void
				  addPointIndex (int data_arg)
			  {
				  leafDataTVector_.push_back (data_arg);
			  }

			  /** \brief Retrieve point index from container. This container stores a vector of point indices.
			  * \return index stored within container.
			  */
			  int
				  getPointIndex ( ) const
			  {
				  return leafDataTVector_.back ();
			  }

			  /** \brief Retrieve point indices from container. This container stores a vector of point indices.
			  * \param[out] data_vector_arg vector of point indices to be stored within data vector
			  */
// 			  void
// 				  getPointIndices (std::vector<int>& data_vector_arg) const
// 			  {
// 				  data_vector_arg.insert (data_vector_arg.end (), leafDataTVector_.begin (), leafDataTVector_.end ());
// 			  }

			  /** \brief Retrieve reference to point indices vector. This container stores a vector of point indices.
			  * \return reference to vector of point indices to be stored within data vector
			  */
			  std::vector<int>*
				  getPointIndices ()
			  {
				  return &leafDataTVector_;
			  }

			  /** \brief Get size of container (number of indices)
			  * \return number of point indices in container.
			  */
			  size_t
				  getSize () const
			  {
				  return leafDataTVector_.size ();
			  }

			  /** \brief Reset leaf node. Clear DataT vector.*/
			  virtual void
				  reset ()
			  {
				  leafDataTVector_.clear ();
			  }

		public:
			VoxelContainerPointIndices *fatherptr;
			pcl::octree::OctreeKey   fatherindice;

			//voxel features
			VoxelFeature feat;

		protected:
			/** \brief Leaf node DataT vector. */
			std::vector<int> leafDataTVector_;


		};

	}
}


#endif