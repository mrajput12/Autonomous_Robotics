import open3d as o3d
import numpy as np
import matplotlib.pyplot as plt
from sklearn.cluster import DBSCAN
import plotly.graph_objects as go
from plotly import colors



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

        plane_model, inliers = pcd.segment_plane(distance_threshold=0.05, ransac_n=3, num_iterations=1000)
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
    def pass_through_fileter(self,points:np.ndarray,view=True,z_min=0, z_max=1.6,x_min=-1,x_max=23) -> np.ndarray:
        """
        Purpose of the function is to filter out the points like croping the point cloud.
        parm: x_min,x_max,y_min,y_max,z_min,z_max are the min and max values for the pass through filter
        :param points: numpy array of point clouds
        :param save_path: save path for the npy file after filtering
        :param save: if you want to save the npy file or want to get the filtered points
        :return: nd.ndarray of filtered points
        """

        mask = (points[:, 2] >= z_min) & (points[:, 2] <= z_max)&(points[:, 0] >= x_min) & (points[:, 0] <= x_max)
        points_z_x_range = points[mask]
        if view==True:
            pcd_filtered = o3d.geometry.PointCloud()
            pcd_filtered.points = o3d.utility.Vector3dVector(points_z_x_range)
            o3d.visualization.draw_geometries([pcd_filtered],window_name="filtered point cloud")
        else:
            print("filtered point cloud is not viewed")
        return points_z_x_range

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
        return all_clusters,cluster_nr,labels
    def draw_labels(self,points,labels):
        """
        Args:
            points: np.array of point clouds


        Returns: point cloud with labels

        """
        #craete color pallet with plotly
        n = len(set(labels))
        cls = colors.sequential.Plasma[0:n]
        fig = go.Figure(data=[go.Scatter3d(x=points[:, 0], y=points[:, 1], z=points[:, 2], mode='markers'
                                           ,marker=dict(color=labels),text=labels)])

        # Show the plot
        fig.show()

    def get_center_of_culster(self,all_clusters,labels):
        """
        Purpose of the function is to get the center of the cluster
        :param cluster_nr: numpy array of cluster
        :return: center of the cluster
        """
        for i in range(len(set(labels))):
            single_cluster=all_clusters[np.where(labels == i)]
            print(f"calcuating dimentions of the cluster Nr.: {i}")
            print(f"##################{i}###################")


            #center = np.mean(cluster_nr, axis=0)
            #getting the min and max of all axis of the cluster
            x_min=np.min(single_cluster[:,0])
            x_max=np.max(single_cluster[:,0])
            y_min=np.min(single_cluster[:,1])
            y_max=np.max(single_cluster[:,1])

            print(f"Cluster Nr {i} min x:{x_min} max x:{x_max} min y:{y_min} max y:{y_max}")






if __name__=="__main__":
    c=Segmentation("/home/souz/UTP/Inventory-management/dev/catkin_ws/src/uav-v01/uav_sim/pointcloud_maps/warehouse_legs.pcd")
    #pp=c.get_planes("/home/souz/UTP/Inventory-management/dev/catkin_ws/src/uav-v01/uav_sim/pointcloud_maps/my_outlier_plans.npy",save=True)
    #c.get_clusters("/home/souz/UTP/Inventory-management/dev/catkin_ws/src/uav-v01/uav_sim/pointcloud_maps/my_outlier_plans.npy", culster_n=4, view_all_clusters=True, view_single_cluster=True)
    np_array_point=np.load("/home/souz/UTP/Inventory-management/dev/catkin_ws/src/uav-v01/uav_sim/pointcloud_maps/my_outlier_plans.npy")
    filtered_cloud=c.pass_through_fileter(np_array_point)
    all_cluster,single_cluster,labels=c.get_clusters(filtered_cloud, culster_n=4, view_all_clusters=True, view_single_cluster=True)
    c.get_center_of_culster(all_cluster,labels)
    c.draw_labels(all_cluster,labels)