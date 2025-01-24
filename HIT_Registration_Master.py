# coding:utf-8
'''
**************************************************
@File   ：HIT_Registration_Master
@IDE    ：PyCharm
@Author ：Tianze Zhang
@Desc   , Main Registration Master Function
@Date   ：2025/1/23 20:48
**************************************************
'''

from Ui_test_vispy import Ui_MainWindow
from Ui_MainWindow import Ui_MainWindow
from PySide6.QtCore import *
from PySide6.QtGui import *
from PySide6.QtWidgets import *
import sys
import vispy.scene
from vispy.scene import visuals
import numpy as np
import open3d as o3d
import copy
import time
import math
import os

import pycpd
from pycpd import RigidRegistration


class IndexController(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # vispy.scene
        self.canvas1 = vispy.scene.SceneCanvas(keys='interactive', show=True)
        self.canvas2 = vispy.scene.SceneCanvas(keys='interactive', show=True)
        self.view1 = self.canvas1.central_widget.add_view()
        self.view2 = self.canvas2.central_widget.add_view()

        # Embed SceneCanvas in UI layouts
        self.source.addWidget(self.canvas1.native)
        self.target.addWidget(self.canvas2.native)

        # Connect buttons to callbacks
        self.pushButton1.clicked.connect(self.showVispySourcePointCloud)
        self.pushButton2.clicked.connect(self.showVispyTargetPointCloud)
        self.pushButton3.clicked.connect(self.selectSourcePointCloudPath)
        self.pushButton4.clicked.connect(self.selectTargetPointCloudPath)
        self.pushButton5.clicked.connect(self.preprocessPointClouds)
        self.pushButton6.clicked.connect(self.showVispyPreprocessedPointClouds)
        self.pushButton7.clicked.connect(self.registrationPointClouds)
        self.pushButton8.clicked.connect(self.showVispyCoarseResult)
        self.pushButton9.clicked.connect(self.showVispyFineResult)
        self.pushButton11.clicked.connect(self.scaleTransformPointCloud)
        self.pushButton12.clicked.connect(self.saveSourcePointCloud)
        self.pushButton13.clicked.connect(self.saveTargetPointCloud)

        # Initialize comboBox contents
        self.comboBox2.clear()
        self.comboBox2.addItem('Please select')
        self.comboBox2.addItems(["Coarse", "Fine"])

    def segmentExample(self, cloud):
        """
        Example plane segmentation function for demonstration only.
        Modify or remove as needed.
        """
        center = cloud.get_center()
        R = cloud.get_rotation_matrix_from_xyz((0, 0, np.pi / 6))
        cloud.rotate(R, center)
        plane_model, inliers = cloud.segment_plane(distance_threshold=1,
                                                   ransac_n=1000,
                                                   num_iterations=500)
        outlier = cloud.select_by_index(inliers, invert=True)
        points = np.asarray(outlier.points)
        # Example region selection
        idx = np.where((points[:, 0] >= 55) & (points[:, 0] <= 85) &
                       (points[:, 1] >= 4) & (points[:, 1] <= 40) &
                       (points[:, 2] >= 110) & (points[:, 2] <= 120))[0]

        inlier_cloud = outlier.select_by_index(idx)
        center = inlier_cloud.get_center()
        R = inlier_cloud.get_rotation_matrix_from_xyz((np.pi, 0, 0))
        inlier_cloud.rotate(R, center)
        return inlier_cloud

    def showVispySourcePointCloud(self):
        """
        Load and display source point cloud
        """
        self.sourcePCD = o3d.io.read_point_cloud(self.sourcePath[0], format='xyz')

        # Example usage: remove ground with segmentation
        # self.sourcePCD = self.segmentExample(self.sourcePCD)

        source_points = np.asarray(self.sourcePCD.points)
        self.sourcePointCount = source_points.size
        self.textEdit9.setText(str(self.sourcePointCount / 3))

        scatter1 = visuals.Markers()
        scatter1.set_data(source_points, edge_color=None, face_color=(1, 1, 0, .5), size=5)

        self.view1.add(scatter1)
        self.view1.camera = 'turntable'
        visuals.XYZAxis(parent=self.view1.scene)

    def showVispyTargetPointCloud(self):
        """
        Load and display target point cloud
        """
        self.targetPCD = o3d.io.read_point_cloud(self.targetPath[0], format='xyz')
        target_points = np.asarray(self.targetPCD.points)
        self.targetPointCount = target_points.size
        self.textEdit10.setText(str(self.targetPointCount / 3))

        # Estimate scale ratio
        len_source = self.computeBBoxScale(self.sourcePCD)
        len_target = self.computeBBoxScale(self.targetPCD)
        self.ratio = self.computeScaleRatio(len_source, len_target)
        self.textEdit3.setText(str(self.ratio))

        scatter2 = visuals.Markers()
        scatter2.set_data(target_points, edge_color=None, face_color=(0, 1, 1, .5), size=5)

        self.view1.add(scatter2)
        self.view1.camera = 'turntable'
        visuals.XYZAxis(parent=self.view1.scene)

    def selectSourcePointCloudPath(self):
        self.sourcePath = QFileDialog.getOpenFileName(self, "Load source point cloud")
        if self.sourcePath != '':
            self.textEdit1.setText(str(self.sourcePath[0]))
        return self.sourcePath

    def selectTargetPointCloudPath(self):
        self.targetPath = QFileDialog.getOpenFileName(self, "Load target point cloud")
        if self.targetPath != '':
            self.textEdit2.setText(str(self.targetPath[0]))
        return self.targetPath

    def computeBBoxScale(self, pcd):
        obb = pcd.get_oriented_bounding_box()
        vertices = np.asarray(obb.get_box_points())
        max_bound = np.asarray(obb.get_max_bound())
        min_bound = np.asarray(obb.get_min_bound())

        for i in range(8):
            if vertices[i][0] == max_bound[0]:
                x_max = vertices[i]
            elif vertices[i][0] == min_bound[0]:
                x_min = vertices[i]

        x_len = math.sqrt(
            (x_max[0] - x_min[0]) ** 2 +
            (x_max[1] - x_min[1]) ** 2 +
            (x_max[2] - x_min[2]) ** 2
        )
        return x_len

    def computeScaleRatio(self, len_source, len_target):
        ratio_val = len_source / len_target
        ratio_val = ('%.4f' % ratio_val)
        return float(ratio_val)

    def scaleTransformPointCloud(self):
        """
        Scale the target point cloud by self.ratio
        """
        self.targetPCD.transform(np.array([
            [self.ratio, 0.0,      0.0,      0.0],
            [0.0,       self.ratio,0.0,      0.0],
            [0.0,       0.0,       self.ratio,0.0],
            [0.0,       0.0,       0.0,       1.0]
        ]))

        source_points = np.asarray(self.sourcePCD.points)
        scatter1 = visuals.Markers()
        scatter1.set_data(source_points, edge_color=None, face_color=(1, 1, 0, .5), size=5)

        target_points = np.asarray(self.targetPCD.points)
        scatter2 = visuals.Markers()
        scatter2.set_data(target_points, edge_color=None, face_color=(0, 1, 1, .5), size=5)

        self.canvas1.central_widget.remove_widget(widget=self.view1)
        self.view1 = self.canvas1.central_widget.add_view()
        self.view1.add(scatter1)
        self.view1.add(scatter2)
        self.view1.camera = 'turntable'
        visuals.XYZAxis(parent=self.view1.scene)

    def preprocessPointClouds(self):
        """
        Apply voxel downsampling (to a target point count) and optional statistical outlier removal
        to both source and target point clouds.
        """
        self.textEdit11.clear()
        target_count = np.array(self.textEdit4.toPlainText(), dtype=np.float32)

        if target_count == 0:
            self.sourcePCD_new = self.sourcePCD
            self.targetPCD_new = self.targetPCD
        else:
            # Source
            voxel = 0.01
            src_down = copy.deepcopy(self.sourcePCD)
            for _ in range(10000):
                temp = src_down.voxel_down_sample(voxel_size=voxel)
                new_size = len(temp.points)
                if new_size <= target_count:
                    src_down = temp
                    break
                voxel += 0.01
            self.sourcePCD_new = src_down

            # Target
            voxel = 0.01
            tgt_down = copy.deepcopy(self.targetPCD)
            for _ in range(10000):
                temp = tgt_down.voxel_down_sample(voxel_size=voxel)
                new_size = len(temp.points)
                if new_size <= target_count:
                    tgt_down = temp
                    break
                voxel += 0.01
            self.targetPCD_new = tgt_down

        if self.radioButton3.isChecked():
            neighbor_val = np.array(self.textEdit5.toPlainText(), dtype=np.float32)
            thresh_val = np.array(self.textEdit6.toPlainText(), dtype=np.float32)

            _, ind_s = self.sourcePCD_new.remove_statistical_outlier(
                nb_neighbors=int(neighbor_val),
                std_ratio=float(thresh_val)
            )
            self.sourcePCD_new = self.sourcePCD_new.select_by_index(ind_s)

            _, ind_t = self.targetPCD_new.remove_statistical_outlier(
                nb_neighbors=int(neighbor_val),
                std_ratio=float(thresh_val)
            )
            self.targetPCD_new = self.targetPCD_new.select_by_index(ind_t)

        self.textEdit11.setText("Done")

    @staticmethod
    def pcaCompute(pcd):
        center, covariance = pcd.compute_mean_and_covariance()
        eigenvectors, _, _ = np.linalg.svd(covariance)
        return eigenvectors, center

    @staticmethod
    def pcaRegistration(P, X):
        """
        Performs PCA-based coarse registration: P -> X
        Returns the transformed point cloud and final transform matrix
        """
        P.paint_uniform_color([1, 0, 0])
        X.paint_uniform_color([0, 1, 0])

        errors = []
        matrices = []
        Up, Cp = IndexController.pcaCompute(P)
        Ux, Cx = IndexController.pcaCompute(X)
        Upcopy = Up

        sign1 = [1, -1, 1, 1, -1, -1, 1, -1]
        sign2 = [1, 1, -1, 1, -1, 1, -1, -1]
        sign3 = [1, 1, 1, -1, 1, -1, -1, -1]
        for nn in range(len(sign3)):
            Up[0] = sign1[nn] * Upcopy[0]
            Up[1] = sign2[nn] * Upcopy[1]
            Up[2] = sign3[nn] * Upcopy[2]
            R0 = np.dot(Ux, np.linalg.inv(Up))
            T0 = Cx - np.dot(R0, Cp)
            T = np.eye(4)
            T[:3, :3] = R0
            T[:3, 3] = T0
            transformed = copy.deepcopy(P).transform(T)
            dists = transformed.compute_point_cloud_distance(X)
            dists = np.asarray(dists)
            mse = np.average(dists)
            errors.append(mse)
            matrices.append(T)

        best_idx = errors.index(min(errors))
        final_T = matrices[best_idx]
        pca_transformed = copy.deepcopy(P).transform(final_T)
        return pca_transformed, final_T

    def registrationPointClouds(self):
        """
        Perform either PCA coarse registration or CPD fine registration
        based on comboBox2 selection.
        """
        if self.radioButton4.isChecked():
            # "Coarse"
            if self.comboBox2.currentIndex() == 1:
                self.finalPCD, self.finalT = IndexController.pcaRegistration(self.sourcePCD_new, self.targetPCD_new)
                dists = self.finalPCD.compute_point_cloud_distance(self.targetPCD_new)
                dists = np.asarray(dists)
                self.rmseCoarse = np.sqrt(np.average(dists * dists))
                self.pushButton10.clicked.connect(self.showCoarseTransformResult)

            # "Fine"
            elif self.comboBox2.currentIndex() == 2:
                reg = RigidRegistration(
                    X=np.asarray(self.targetPCD_new.points),
                    Y=np.asarray(self.finalPCD.points),
                    max_iterations=100,
                    tolerance=0.001
                )
                # If pycpd returns (R, t, s), we can discard s or use it as needed
                transformed_points, (R, t, _) = reg.register()

                self.resultPCD = copy.deepcopy(self.finalPCD)
                self.resultPCD.points = o3d.utility.Vector3dVector(transformed_points)
                dists = self.resultPCD.compute_point_cloud_distance(self.targetPCD_new)
                dists = np.asarray(dists)
                self.rmseFine = np.sqrt(np.average(dists * dists))
                self.pushButton10.clicked.connect(self.showFineTransformResult)

    def showCoarseTransformResult(self):
        self.textEdit7.setText(str(self.finalT))
        self.textEdit8.setText(str(self.rmseCoarse))

    def showFineTransformResult(self):
        self.textEdit8.clear()
        self.textEdit7.clear()
        self.textEdit8.setText(str(self.rmseFine))

    def showVispyPreprocessedPointClouds(self):
        source_points = np.asarray(self.sourcePCD_new.points)
        self.sourcePointCountNew = source_points.size
        self.textEdit9.clear()
        self.textEdit9.setText(str(self.sourcePointCountNew / 3))
        scatter1 = visuals.Markers()
        scatter1.set_data(source_points, edge_color=None, face_color=(1, 1, 0, .5), size=5)

        target_points = np.asarray(self.targetPCD_new.points)
        self.targetPointCountNew = target_points.size
        self.textEdit10.clear()
        self.textEdit10.setText(str(self.targetPointCountNew / 3))
        scatter2 = visuals.Markers()
        scatter2.set_data(target_points, edge_color=None, face_color=(0, 1, 1, .5), size=5)

        self.canvas1.central_widget.remove_widget(widget=self.view1)
        self.view1 = self.canvas1.central_widget.add_view()
        self.view1.add(scatter1)
        self.view1.add(scatter2)
        self.view1.camera = 'turntable'
        visuals.XYZAxis(parent=self.view1.scene)

    def showVispyCoarseResult(self):
        final_points = np.asarray(self.finalPCD.points)
        scatter1 = visuals.Markers()
        scatter1.set_data(final_points, edge_color=None, face_color=(1, 1, 0, .5), size=5)

        target_points = np.asarray(self.targetPCD_new.points)
        scatter2 = visuals.Markers()
        scatter2.set_data(target_points, edge_color=None, face_color=(0, 1, 1, .5), size=5)

        self.canvas2.central_widget.remove_widget(widget=self.view2)
        self.view2 = self.canvas2.central_widget.add_view()
        self.view2.add(scatter1)
        self.view2.add(scatter2)
        self.view2.camera = 'turntable'
        visuals.XYZAxis(parent=self.view2.scene)

    def showVispyFineResult(self):
        result_points = np.asarray(self.resultPCD.points)
        scatter1 = visuals.Markers()
        scatter1.set_data(result_points, edge_color=None, face_color=(1, 1, 0, .5), size=5)

        target_points = np.asarray(self.targetPCD_new.points)
        scatter2 = visuals.Markers()
        scatter2.set_data(target_points, edge_color=None, face_color=(0, 1, 1, .5), size=5)

        self.canvas2.central_widget.remove_widget(widget=self.view2)
        self.view2 = self.canvas2.central_widget.add_view()
        self.view2.add(scatter1)
        self.view2.add(scatter2)
        self.view2.camera = 'turntable'
        visuals.XYZAxis(parent=self.view2.scene)

    def saveSourcePointCloud(self):
        save_dir = QFileDialog.getExistingDirectory(self, "Save source point cloud")
        if save_dir:
            save_path = '/'.join((save_dir, 'source_after_registration.pcd'))
            # If you want to save the final fine-registered point cloud, change to self.resultPCD
            # To save the coarse-registered one, use self.finalPCD
            o3d.io.write_point_cloud(save_path, self.finalPCD,
                                     write_ascii=False,
                                     compressed=False,
                                     print_progress=False)

    def saveTargetPointCloud(self):
        save_dir = QFileDialog.getExistingDirectory(self, "Save target point cloud")
        if save_dir:
            save_path = '/'.join((save_dir, 'target_after_preprocessing.pcd'))
            o3d.io.write_point_cloud(save_path, self.targetPCD_new,
                                     write_ascii=False,
                                     compressed=False,
                                     print_progress=False)


if __name__ == '__main__':
    app = QApplication([])
    main = IndexController()
    main.show()
    sys.exit(app.exec())
