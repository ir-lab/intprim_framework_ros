#   @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University
import datetime
import hinton_diagram as hd
import matplotlib
matplotlib.use("Qt5Agg")
import matplotlib.pyplot as plt
import numpy as np
import os
import PyQt5.QtCore
import PyQt5.QtWidgets
import rospy
import scipy.optimize
import sklearn.metrics
import sklearn.model_selection

import intprim as bip

import intprim_framework_ros.msg
import intprim_framework_ros.srv



class ExportNoiseDialog(PyQt5.QtWidgets.QDialog):
    def __init__(self, export_path, bias_value):
        super(ExportNoiseDialog, self).__init__()

        self.init_widget_ui(export_path, bias_value)

    def init_widget_ui(self, export_path, bias_value):
        self.bias_value = PyQt5.QtWidgets.QLineEdit(str(bias_value))
        self.export_path = PyQt5.QtWidgets.QLineEdit(export_path)
        self.button_box = PyQt5.QtWidgets.QDialogButtonBox(PyQt5.QtWidgets.QDialogButtonBox.Ok | PyQt5.QtWidgets.QDialogButtonBox.Cancel)
        self.button_box.accepted.connect(self.accept)
        self.button_box.rejected.connect(self.reject)

        layout = PyQt5.QtWidgets.QFormLayout()
        layout.setFieldGrowthPolicy(PyQt5.QtWidgets.QFormLayout.AllNonFixedFieldsGrow)
        layout.addRow('Bias', self.bias_value)
        layout.addRow('Export Path', self.export_path)
        layout.addWidget(self.button_box)



        self.setLayout(layout)

        self.setWindowTitle("Export Observation Noise")
        self.setMinimumWidth(500)

class ViewMinMaxDialog(PyQt5.QtWidgets.QDialog):
    def __init__(self, values):
        super(ViewMinMaxDialog, self).__init__()

        self.init_widget_ui(values)

    def init_widget_ui(self, values):
        widget = PyQt5.QtWidgets.QWidget()
        layout = PyQt5.QtWidgets.QGridLayout()

        row_idx = 0
        col_idx = 0

        # Set up headers
        layout.addWidget(PyQt5.QtWidgets.QLabel("Name"), row_idx, 0)
        layout.addWidget(PyQt5.QtWidgets.QLabel("Min Value"), row_idx, 1)
        layout.addWidget(PyQt5.QtWidgets.QLabel("Max Value"), row_idx, 2)
        layout.addWidget(PyQt5.QtWidgets.QLabel("Avg Value"), row_idx, 3)
        row_idx += 1
        line = PyQt5.QtWidgets.QFrame()
        line.setFrameShape(PyQt5.QtWidgets.QFrame.HLine)
        line.setFrameShadow(PyQt5.QtWidgets.QFrame.Sunken)
        layout.addWidget(line, row_idx, 0, 1, 4)
        row_idx += 1

        for modality in values:
            for dof in modality:
                layout.addWidget(PyQt5.QtWidgets.QLabel(dof[0]), row_idx, 0)
                label = PyQt5.QtWidgets.QLabel(str(dof[1]))
                label.setTextInteractionFlags(PyQt5.QtCore.Qt.TextSelectableByMouse)
                layout.addWidget(label, row_idx, 1)
                label = PyQt5.QtWidgets.QLabel(str(dof[2]))
                label.setTextInteractionFlags(PyQt5.QtCore.Qt.TextSelectableByMouse)
                layout.addWidget(label, row_idx, 2)
                label = PyQt5.QtWidgets.QLabel(str(dof[3]))
                label.setTextInteractionFlags(PyQt5.QtCore.Qt.TextSelectableByMouse)
                layout.addWidget(label, row_idx, 3)
                row_idx += 1

            line = PyQt5.QtWidgets.QFrame()
            line.setFrameShape(PyQt5.QtWidgets.QFrame.HLine)
            line.setFrameShadow(PyQt5.QtWidgets.QFrame.Sunken)
            layout.addWidget(line, row_idx, 0, 1, 4)
            row_idx += 1

        self.setLayout(layout)

        self.setWindowTitle("View Min/Max")
        self.setMinimumWidth(500)

class InteractionPrimitiveWidget(PyQt5.QtWidgets.QWidget):
    DEMONSTRATION_TREE_NAME_IDX    = 0
    DEMONSTRATION_TREE_SAMPLE_IDX  = 1
    DEMONSTRATION_TREE_FEATURE_IDX = 2

    def __init__(self, parent):
        super(InteractionPrimitiveWidget, self).__init__(parent)

        self.parent = parent

        self.init_data_profile()
        self.init_widget_ui()

        self.add_demonstration_service = rospy.ServiceProxy("/ip/addDemonstration", intprim_framework_ros.srv.AddDemonstration)
        self.compute_standardization_service = rospy.ServiceProxy("/ip/computeStandardization", intprim_framework_ros.srv.ComputeStandardization)
        self.get_approx_trajectory_service = rospy.ServiceProxy("/ip/getApproximateTrajectory", intprim_framework_ros.srv.GetApproximateTrajectory)
        self.get_distribution_service = rospy.ServiceProxy("/ip/getDistribution", intprim_framework_ros.srv.GetDistribution)
        self.get_distribution_parameters_service = rospy.ServiceProxy("/ip/getDistributionParameters", intprim_framework_ros.srv.GetDistributionParameters)
        self.get_mean_trajectory_service = rospy.ServiceProxy("/ip/getMeanTrajectory", intprim_framework_ros.srv.GetMeanTrajectory)
        self.export_data_service = rospy.ServiceProxy("/ip/exportData", intprim_framework_ros.srv.ExportData)
        self.add_basis_selection_demonstration_service = rospy.ServiceProxy("/ip/addBasisSelectionDemonstration", intprim_framework_ros.srv.AddBasisSelectionDemonstration)
        self.perform_basis_selection_service = rospy.ServiceProxy("/ip/performBasisSelection", intprim_framework_ros.srv.PerformBasisSelection)
        self.get_basis_error_service = rospy.ServiceProxy("/ip/getBasisError", intprim_framework_ros.srv.GetBasisError)
        self.initialize_service = rospy.ServiceProxy("/ip/initializeState", intprim_framework_ros.srv.InitializeState)

        self.controller_publisher = rospy.Publisher(rospy.get_param("control/control_topic"), intprim_framework_ros.msg.Trajectory, queue_size = 1)

    def init_data_profile(self):
        self.bip_parameters = {}
        for bip_params in rospy.get_param("bip"):
            self.bip_parameters[bip_params["id"]] = bip_params

        self.data_profile = {}
        self.data_active = {}
        self.data_generate = {}
        self.test_directory = []

        for bip_params in rospy.get_param("bip"):
            profile = []
            new_active_dofs = []
            old_active_dofs = []
            new_generate_dofs = []
            old_generate_dofs = []

            self.test_directory.append(bip_params["mip_test_directory"])

            new_idx = 0
            for modality_params in bip_params["modalities"]:
                if(modality_params["active"]):
                    group = []
                    for old_idx, name in zip(range(modality_params["indices"][0], modality_params["indices"][1]), modality_params["dof_names"]):
                        group.append((new_idx, old_idx, name))
                        new_active_dofs.append(new_idx)
                        old_active_dofs.append(old_idx)
                        if(modality_params["generate"]):
                            new_generate_dofs.append(new_idx)
                            old_generate_dofs.append(old_idx)
                        new_idx += 1

                    profile.append(group)

            self.data_profile[bip_params["id"]] = profile
            # This is different from bip service's active/generate.
            # There, generate/active are created via set difference.
            # Here, active is a superset.
            # Active here is really analogous to dof_indices in bip service.
            # Perhaps rename one or other.
            self.data_active[bip_params["id"]] = (new_active_dofs, old_active_dofs)
            self.data_generate[bip_params["id"]] = (new_generate_dofs, old_generate_dofs)

    def init_widget_ui(self):
        self.widget_layout = PyQt5.QtWidgets.QVBoxLayout()

        self.widget_layout.addWidget(self.create_interaction_selection_widget())
        self.widget_layout.addWidget(self.create_demonstration_widget())

        self.setLayout(self.widget_layout)

    def set_busy(self):
        for widget in self.findChildren(PyQt5.QtWidgets.QPushButton):
            widget.setEnabled(False)

    def set_ready(self):
        for widget in self.findChildren(PyQt5.QtWidgets.QPushButton):
            widget.setEnabled(True)

    @PyQt5.QtCore.pyqtSlot()
    def demonstration_tree_callback(self):
        self.plot_demo_button.setEnabled(True)
        self.plot_approx_demo_button.setEnabled(True)

        for item in self.demonstration_tree.selectionModel().selectedRows():
            selected_idx = item.row()

            if(selected_idx not in self.trained_demonstration_indices):
                self.train_demo_button.setEnabled(True)
                return

        self.train_demo_button.setEnabled(False)

    @PyQt5.QtCore.pyqtSlot()
    def demonstration_train_callback(self):
        self.set_busy()
        if(self.bip_parameters[int(self.interaction_selector.currentData())]["scale_observations"]):
            for item in self.demonstration_tree.selectionModel().selectedRows():
                selected_idx = item.row()

                self.standardize_demonstration_data(selected_idx)

        for item in self.demonstration_tree.selectionModel().selectedRows():
            selected_idx = item.row()

            self.train_demonstration_data(selected_idx)
            self.trained_demonstration_indices.append(selected_idx)

        self.set_ready()

    @PyQt5.QtCore.pyqtSlot()
    def demonstration_plot_callback(self):
        for item in self.demonstration_tree.selectionModel().selectedRows():
            selected_idx = item.row()

            self.plot_demonstration_data(self.demonstration_data[selected_idx], old_indices = True)

    @PyQt5.QtCore.pyqtSlot()
    def approx_demonstration_plot_callback(self):
        for item in self.demonstration_tree.selectionModel().selectedRows():
            selected_idx = item.row()

            # Get approximate data
            approx_data = self.get_approx_demonstration_data(selected_idx)

            self.plot_demonstration_data(approx_data, old_indices = False)

    @PyQt5.QtCore.pyqtSlot()
    def demonstration_select_callback(self):
        options = PyQt5.QtWidgets.QFileDialog.Options()
        options |= PyQt5.QtWidgets.QFileDialog.DontUseNativeDialog
        files, _ = PyQt5.QtWidgets.QFileDialog.getOpenFileNames(self,"QFileDialog.getOpenFileNames()", self.test_directory[0],"CSV Files (*.csv)", options = options)
        #"All Files (*);;CSV Files (*.csv)"

        phase_velocities = []

        for file_name in files:
            idx = self.demonstration_tree.model().rowCount()
            self.demonstration_data.append(self.load_demonstration_data(file_name))

            self.demonstration_tree.model().insertRow(idx)
            self.demonstration_tree.model().setData(
                self.demonstration_tree.model().index(idx, self.DEMONSTRATION_TREE_NAME_IDX),
                file_name)
            self.demonstration_tree.model().setData(
                self.demonstration_tree.model().index(idx, self.DEMONSTRATION_TREE_SAMPLE_IDX),
                str(self.demonstration_data[idx].shape[0]))
            self.demonstration_tree.model().setData(
                self.demonstration_tree.model().index(idx, self.DEMONSTRATION_TREE_FEATURE_IDX),
                str(self.demonstration_data[idx].shape[1]))

            phase_velocities.append(1.0 / self.demonstration_data[idx].shape[0])

        phase_mean = np.mean(phase_velocities)
        phase_var = np.var(phase_velocities)

        self.phase_velocity_label.setText(str(phase_mean))
        self.phase_variance_label.setText(str(phase_var))

    @PyQt5.QtCore.pyqtSlot()
    def get_demonstration_distribution_callback(self):
        mean, upper, lower = self.get_demonstration_distribution()

        self.plot_demonstration_distribution(mean, upper, lower)

    @PyQt5.QtCore.pyqtSlot()
    def get_hinton_callback(self):
        mean, covariance = self.get_distribution_parameters()

        self.plot_hinton(covariance)

    @PyQt5.QtCore.pyqtSlot()
    def publish_mean_trajectory_callback(self):
        mean_message = self.get_mean_trajectory_message(False, 50)

        self.controller_publisher.publish(mean_message)

    @PyQt5.QtCore.pyqtSlot()
    def export_data_callback(self):
        response = self.export_data_service(int(self.interaction_selector.currentData()), self.bip_parameters[int(self.interaction_selector.currentData())]["import_data"])

    @PyQt5.QtCore.pyqtSlot()
    def export_noise_callback(self):
        if(len(self.demonstration_tree.selectionModel().selectedRows()) == 0):
            print("Must select demonstrations to calculate noise!")
            return

        export_dialog = ExportNoiseDialog(self.bip_parameters[int(self.interaction_selector.currentData())]["observation_noise"], 1.0)
        if export_dialog.exec_():
            self.export_noise(float(export_dialog.bias_value.text()), export_dialog.export_path.text())

    def export_noise(self, bias_value, path):
        for item in self.demonstration_tree.selectionModel().selectedRows():
            selected_idx = item.row()

            self.add_basis_selection_demonstration_data(selected_idx)

        noise_matrix = self.get_basis_error(float(bias_value))
        np.savetxt(path, noise_matrix, delimiter = ",")

        print("Observation noise exported to: " + path)

    @PyQt5.QtCore.pyqtSlot()
    def basis_selection_callback(self):
        for item in self.demonstration_tree.selectionModel().selectedRows():
            selected_idx = item.row()

            self.add_basis_selection_demonstration_data(selected_idx)

        self.perform_basis_selection()

    @PyQt5.QtCore.pyqtSlot()
    def view_minmax_callback(self):
        values = []
        # Calculate min, max, avg values for each DoF and group by modality
        for modality in self.data_profile[int(self.interaction_selector.currentData())]:
            modality_values = []
            for new_idx, old_idx, dof_name in modality:
                min_val = np.inf
                max_val = -np.inf
                avg_val = None
                #if(not np.any(data[old_idx])):
                for item in self.demonstration_tree.selectionModel().selectedRows():
                    selected_idx = item.row()

                    min_val = np.min([min_val, np.min(self.demonstration_data[selected_idx][:, old_idx])])
                    max_val = np.max([max_val, np.max(self.demonstration_data[selected_idx][:, old_idx])])
                    if(avg_val is None):
                        avg_val = np.mean(self.demonstration_data[selected_idx][:, old_idx])
                    else:
                        avg_val = np.mean([avg_val, np.mean(self.demonstration_data[selected_idx][:, old_idx])])
                modality_values.append((dof_name, min_val, max_val, avg_val))
            values.append(modality_values)

        self.minmax_dialog = ViewMinMaxDialog(values)
        #minmax_dialog.exec_()
        # Use show instead of exec() so we can take focus off of pop up window (useful for comparison to plots).
        self.minmax_dialog.show()

    def get_basis_error(self, bias = 1e0):
        response = self.get_basis_error_service(int(self.interaction_selector.currentData()))

        observation_noise = np.array(response.covariance, dtype = np.float64)
        observation_noise = np.reshape(observation_noise, (len(observation_noise) / response.stride, response.stride))

        observation_noise[np.diag_indices(observation_noise.shape[0])] *= bias

        return observation_noise

    def standardize_demonstration_data(self, demonstration_idx):
        response = self.compute_standardization_service(int(self.interaction_selector.currentData()), intprim_framework_ros.msg.Trajectory(self.demonstration_data[demonstration_idx].shape[1], self.demonstration_data[demonstration_idx].flatten().tolist()))

    def train_demonstration_data(self, demonstration_idx):
        response = self.add_demonstration_service(int(self.interaction_selector.currentData()), intprim_framework_ros.msg.Trajectory(self.demonstration_data[demonstration_idx].shape[1], self.demonstration_data[demonstration_idx].flatten().tolist()))

    def get_approx_demonstration_data(self, demonstration_idx):
        response = self.get_approx_trajectory_service(int(self.interaction_selector.currentData()), intprim_framework_ros.msg.Trajectory(self.demonstration_data[demonstration_idx].shape[1], self.demonstration_data[demonstration_idx].flatten().tolist()), 200)

        data = np.array(response.generated_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / response.generated_trajectory.stride, response.generated_trajectory.stride))

        return data

    def initialize_state(self):
        self.initialize_service()

    def add_basis_selection_demonstration_data(self, demonstration_idx):
        response = self.add_basis_selection_demonstration_service(int(self.interaction_selector.currentData()), intprim_framework_ros.msg.Trajectory(self.demonstration_data[demonstration_idx].shape[1], self.demonstration_data[demonstration_idx].flatten().tolist()))

    def perform_basis_selection(self):
        response = self.perform_basis_selection_service(int(self.interaction_selector.currentData()))

    def get_mean_trajectory_message(self, full_trajectory = False, num_samples = 100):
        response = self.get_mean_trajectory_service(int(self.interaction_selector.currentData()), full_trajectory, num_samples)

        return response.generated_trajectory

    def get_demonstration_distribution(self):
        response = self.get_distribution_service(int(self.interaction_selector.currentData()))

        return (np.reshape(response.mean_trajectory.data, (len(response.mean_trajectory.data) / response.mean_trajectory.stride, response.mean_trajectory.stride)),
            np.reshape(response.upper_std_dev.data, (len(response.upper_std_dev.data) / response.upper_std_dev.stride, response.upper_std_dev.stride)),
            np.reshape(response.lower_std_dev.data, (len(response.lower_std_dev.data) / response.lower_std_dev.stride, response.lower_std_dev.stride)))

    def get_distribution_parameters(self):
        response = self.get_distribution_parameters_service(int(self.interaction_selector.currentData()))
        return (
            np.array(response.mean),
            np.reshape(response.covariance, (len(response.covariance) / len(response.mean), len(response.mean))))


    def is_data_valid(self, data):
        for modality in self.data_profile[int(self.interaction_selector.currentData())]:
            for new_idx, old_idx, dof_name in modality:
                if(not np.any(data[old_idx])):
                    return False

        return True

    def scale_data(self, data):
        return data

    def load_demonstration_data(self, file_name):
        data = np.loadtxt(file_name, delimiter = ",")

        # Filter out leading zero values caused from reading from rosbags.
        skip_idx = 0
        if(self.trimming_checkbox.isChecked()):
            for x in data:
                if(not self.is_data_valid(x)):
                    skip_idx += 1
                else:
                    break

        if(self.scaling_checkbox.isChecked()):
            data = self.scale_data(data)

        return data[skip_idx:, :]

    def plot_demonstration_group(self, figure, demonstration_data, group_idx, label, old_indices):
        num_col = 2.0
        num_row = np.ceil(len(self.data_profile[int(self.interaction_selector.currentData())][group_idx]) / num_col)

        for index, dof in enumerate(self.data_profile[int(self.interaction_selector.currentData())][group_idx]):
            dof_index = 0
            if(old_indices):
                dof_index = 1

            sub_plot = figure.add_subplot(num_row, num_col, index + 1)
            domain = np.linspace(0, 1, demonstration_data.shape[0])
            sub_plot.plot(domain, demonstration_data[:, dof[dof_index]], label = label)
            sub_plot.set_title(dof[2])

    def plot_demonstration_data(self, demonstration_data, secondary_data = None, old_indices = True):
        for group_idx in range(len(self.data_profile[int(self.interaction_selector.currentData())])):
            group_figure = plt.figure()

            self.plot_demonstration_group(group_figure, demonstration_data, group_idx, "actual" if group_idx == 0 else None, old_indices = old_indices)
            if(secondary_data is not None):
                self.plot_demonstration_group(group_figure, secondary_data, group_idx, "inferred" if group_idx == 0 else None, old_indices = old_indices)
                group_figure.legend()

            group_figure.show()

    def plot_distribution_group(self, figure, mean, upper, lower, group_idx):
        num_col = 2.0
        num_row = np.ceil(len(self.data_profile[int(self.interaction_selector.currentData())][group_idx]) / num_col)

        for index, dof in enumerate(self.data_profile[int(self.interaction_selector.currentData())][group_idx]):
            sub_plot = figure.add_subplot(num_row, num_col, index + 1)
            domain = np.linspace(0, 1, mean.shape[0])
            sub_plot.fill_between(domain, upper[:, dof[0]], lower[:, dof[0]], color = "#ccf5ff")
            sub_plot.plot(domain, mean[:, dof[0]], color = "#000000")
            sub_plot.set_title(dof[2])

    def plot_demonstration_distribution(self, mean, upper, lower):
        for group_idx in range(len(self.data_profile[int(self.interaction_selector.currentData())])):
            group_figure = plt.figure()

            self.plot_distribution_group(group_figure, mean, upper, lower, group_idx)

            #group_figure.tight_layout()

            group_figure.show()

    def plot_hinton(self, covariance):
        hd.hinton_fast(covariance)

    def create_interaction_selection_widget(self):
        widget = PyQt5.QtWidgets.QWidget()
        layout = PyQt5.QtWidgets.QHBoxLayout()

        layout.addWidget(PyQt5.QtWidgets.QLabel("Select interaction:"))

        self.interaction_selector = PyQt5.QtWidgets.QComboBox()
        for interaction_id, bip_params in self.bip_parameters.iteritems():
            self.interaction_selector.addItem(str(bip_params["name"]), str(interaction_id))

        layout.addWidget(self.interaction_selector)

        layout.addStretch(1)

        self.scaling_checkbox = PyQt5.QtWidgets.QCheckBox("Scaling")
        self.scaling_checkbox.setChecked(False)
        layout.addWidget(self.scaling_checkbox)

        self.trimming_checkbox = PyQt5.QtWidgets.QCheckBox("Trimming")
        self.trimming_checkbox.setChecked(False)
        layout.addWidget(self.trimming_checkbox)

        self.plot_checkbox = PyQt5.QtWidgets.QCheckBox("Plot Eval")
        self.plot_checkbox.setChecked(False)
        layout.addWidget(self.plot_checkbox)

        widget.setLayout(layout)

        return widget

    def create_demonstration_widget(self):
        self.demonstration_data = []
        self.trained_demonstration_indices = []

        widget = PyQt5.QtWidgets.QGroupBox("Demonstration Analysis")
        layout = PyQt5.QtWidgets.QHBoxLayout()

        selection_group = PyQt5.QtWidgets.QWidget()
        selection_layout = PyQt5.QtWidgets.QVBoxLayout()

        selection_layout.addWidget(self.create_demonstration_tree_widget())

        phase_group = PyQt5.QtWidgets.QWidget()
        phase_layout = PyQt5.QtWidgets.QHBoxLayout()

        phase_layout.addWidget(PyQt5.QtWidgets.QLabel("Phase Velocity:"))
        self.phase_velocity_label = PyQt5.QtWidgets.QLabel("N/A")
        self.phase_velocity_label.setTextInteractionFlags(PyQt5.QtCore.Qt.TextSelectableByMouse)
        phase_layout.addWidget(self.phase_velocity_label)
        phase_layout.addWidget(PyQt5.QtWidgets.QLabel("Phase Variance:"))
        self.phase_variance_label = PyQt5.QtWidgets.QLabel("N/A")
        self.phase_variance_label.setTextInteractionFlags(PyQt5.QtCore.Qt.TextSelectableByMouse)
        phase_layout.addWidget(self.phase_variance_label)
        self.view_minmax_button = PyQt5.QtWidgets.QPushButton("View Min/Max")
        self.view_minmax_button.clicked.connect(self.view_minmax_callback)
        phase_layout.addWidget(self.view_minmax_button)
        phase_layout.addStretch(1)
        phase_group.setLayout(phase_layout)

        selection_layout.addWidget(phase_group)
        selection_group.setLayout(selection_layout)
        layout.addWidget(selection_group)

        demonstration_tree_group = PyQt5.QtWidgets.QWidget()
        demonstration_tree_layout = PyQt5.QtWidgets.QVBoxLayout()

        select_demo_button = PyQt5.QtWidgets.QPushButton("Select Demonstration(s)")
        select_demo_button.clicked.connect(self.demonstration_select_callback)
        demonstration_tree_layout.addWidget(select_demo_button)

        self.plot_demo_button = PyQt5.QtWidgets.QPushButton("Plot Demonstration(s)")
        self.plot_demo_button.clicked.connect(self.demonstration_plot_callback)
        demonstration_tree_layout.addWidget(self.plot_demo_button)
        self.plot_demo_button.setEnabled(False)

        self.plot_approx_demo_button = PyQt5.QtWidgets.QPushButton("Plot Approx. Demonstration(s)")
        self.plot_approx_demo_button.clicked.connect(self.approx_demonstration_plot_callback)
        demonstration_tree_layout.addWidget(self.plot_approx_demo_button)
        self.plot_approx_demo_button.setEnabled(False)

        self.train_demo_button = PyQt5.QtWidgets.QPushButton("Train Demonstration(s)")
        self.train_demo_button.clicked.connect(self.demonstration_train_callback)
        demonstration_tree_layout.addWidget(self.train_demo_button)
        self.train_demo_button.setEnabled(False)

        distribution_button = PyQt5.QtWidgets.QPushButton("Plot Distribution")
        distribution_button.clicked.connect(self.get_demonstration_distribution_callback)
        demonstration_tree_layout.addWidget(distribution_button)

        hinton_button = PyQt5.QtWidgets.QPushButton("View Correlations")
        hinton_button.clicked.connect(self.get_hinton_callback)
        demonstration_tree_layout.addWidget(hinton_button)

        publish_mean_button = PyQt5.QtWidgets.QPushButton("Publish Mean Demonstration")
        publish_mean_button.clicked.connect(self.publish_mean_trajectory_callback)
        demonstration_tree_layout.addWidget(publish_mean_button)

        export_data_button = PyQt5.QtWidgets.QPushButton("Export Primitive")
        export_data_button.clicked.connect(self.export_data_callback)
        demonstration_tree_layout.addWidget(export_data_button)

        export_noise_button = PyQt5.QtWidgets.QPushButton("Export Observation Noise")
        export_noise_button.clicked.connect(self.export_noise_callback)
        demonstration_tree_layout.addWidget(export_noise_button)

        basis_selection_button = PyQt5.QtWidgets.QPushButton("Perform Basis Selection")
        basis_selection_button.clicked.connect(self.basis_selection_callback)
        demonstration_tree_layout.addWidget(basis_selection_button)

        demonstration_tree_group.setLayout(demonstration_tree_layout)
        layout.addWidget(demonstration_tree_group)

        widget.setLayout(layout)

        return widget

    def create_demonstration_tree_widget(self):
        self.demonstration_tree = PyQt5.QtWidgets.QTreeView()
        self.demonstration_tree.setRootIsDecorated(False)
        self.demonstration_tree.setAlternatingRowColors(True)
        self.demonstration_tree.clicked.connect(self.demonstration_tree_callback)
        self.demonstration_tree.setSelectionMode(PyQt5.QtWidgets.QAbstractItemView.ExtendedSelection)

        model = PyQt5.QtGui.QStandardItemModel(0, 3)
        model.setHeaderData(self.DEMONSTRATION_TREE_NAME_IDX, PyQt5.QtCore.Qt.Horizontal, "Name")
        model.setHeaderData(self.DEMONSTRATION_TREE_SAMPLE_IDX, PyQt5.QtCore.Qt.Horizontal, "Samples")
        model.setHeaderData(self.DEMONSTRATION_TREE_FEATURE_IDX, PyQt5.QtCore.Qt.Horizontal, "Features")

        self.demonstration_tree.setModel(model)

        self.demonstration_tree.header().setStretchLastSection(False);
        self.demonstration_tree.header().setSectionResizeMode(0, PyQt5.QtWidgets.QHeaderView.Stretch);

        return self.demonstration_tree
