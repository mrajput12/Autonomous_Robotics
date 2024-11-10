import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN


class Segmentation():
    """
    class for getting the segmentation clusters
    Args: requires path to pcd.
    """
    def __init__(self,pcd_path):
        self.pcd_path=pcd_path
        return
    def get_planes(self,save_path:str,save=True) -> None:
        """
        Parm:
        save_path= path to save the npy file of plane segmentation#
        to avoid time and again model application

        Returns: np array of point cloud of plan based
        segmentation using ransac model
        if save == True model also saves the np point clouds
        on local file path OR
        np array of point clouds.

        """
        pcd = o3d.io.read_point_cloud(self.pcd_path)

        plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
        [a, b, c, d] = plane_model
        print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        #inlier_cloud = pcd.select_by_index(inliers)
        outlier_cloud = pcd.select_by_index(inliers, invert=True)
        points = np.asarray(outlier_cloud.points).copy()
        #check if given extension is ok
        assert save_path.endswith('.npy'), f"Invalid file extension {save_path}, expected '.npy'"

        if save==True:
            np.save(save_path,points)
            print(f"The npy point of the plans are saved into::> {save_path}")
        elif save==False:
            return points
        return

    def get_clusters(self, npy_cloud:None, culster_n:int, view_single_cluster=True, view_all_clusters=True):
        """

        Args:
            npy_cloud: is either np.array of pointclouds as object
            or loading .npy file form loacal storage
            if you don,t have .npy file saved,
                either run: Class Segment.get_plan()
                the function cloud either give you point cloud object or npy file saved

        Returns: two values; all_clusters point as np.array and single clusters as array

        """
        points=None
        if type(npy_cloud) == str:
            print(f"following path to npy cloud is given::>{npy_cloud}")
            points=np.asarray(np.load(npy_cloud)).copy()
        else:
            print(f"following type cloud ::>{type(npy_cloud)} is given")
            points=npy_cloud
        #fitting DBSCAN model
        model = DBSCAN(eps=0.05, min_samples=6)
        model.fit(points)

        # Get labels:
        labels = model.labels_
        cluster_nr = points[np.where(labels == culster_n)]
        all_clusters=points
        print(f"given:{culster_n} has length: {len(cluster_nr)} and look like {cluster_nr}")
        n_clusters = len(set(labels))
        print('model has following number of cluster', n_clusters)
        # Mapping the labels classes to a color map:
        colors = plt.get_cmap("tab20")(labels / (n_clusters if n_clusters > 0 else 1))
        # Attribute to noise the black color:
        colors[labels < 0] = 0
        # Update points colors:
        #outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        #outlier_cloud.colors = o3d.utility.Vector3dVector(colors[:, :3])
        if view_single_cluster==True:
            pcd_projected = o3d.geometry.PointCloud()  # create point cloud object
            pcd_projected.points = o3d.utility.Vector3dVector(cluster_nr)  # set pcd_np as the point cloud points
            o3d.visualization.draw_geometries([pcd_projected],window_name=f"segments for cluster Nr. >>{culster_n}")
        if view_all_clusters==True:

            pcd_projected = o3d.geometry.PointCloud()  # create point cloud object

            pcd_projected.points = o3d.utility.Vector3dVector(points)  # set pcd_np as the point cloud points
            pcd_projected.colors=o3d.utility.Vector3dVector(colors[:, :3])
            o3d.visualization.draw_geometries([pcd_projected], window_name=f"all {n_clusters} segments.")
        return all_clusters,cluster_nr,


if __name__=="__main__":
    c=Segmentation("/home/faraz/ws_ross/src/uav-v01/uav_sim/cloud_label/pointclouds/filterdwarehouse.pcd")
    #pp=c.get_planes("/home/faraz/my_outlier_plans.npy",save=False)
    c.get_clusters("/home/faraz/ws_ross/src/uav-v01/uav_sim/cloud_label/pointclouds/my_outlier_plans.npy", culster_n=4, view_all_clusters=False, view_single_cluster=True)

