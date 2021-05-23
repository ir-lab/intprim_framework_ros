#!/usr/bin/env python
#   @author Joseph Campbell <jacampb1@asu.edu>, Interactive Robotics Lab, Arizona State University
##
#   @addtogroup intprim_service
#   @{
#
#   A Python ROS service wrapper for facilitating communication between interaction_core (C++) and intprim (Python).

import datetime
import intprim as bip
import intprim.basis.gaussian_model
import intprim.basis.mixture_model
import intprim.basis.polynomial_model
import intprim.basis.sigmoidal_model
import intprim.basis.selection
# import intprim.util.gaussian
import intprim_framework_ros.msg
import intprim_framework_ros.srv
import itertools
import numpy as np
import rospy
import sklearn.metrics
import sklearn.mixture
import analysis.stat_collector

VERBOSE = False

##
#   IntPrimService acts as a middle-man between the interaction_core (C++) and intprim (Python) libraries.
#
#   It provides ROS services for:
#   * adding demonstrations
#   * performing inference
#   * evaluating demonstrations
#   * choosing a basis space
#   * calculating the number of components for a mixture model prior
#   * exporting analysis statistics
#   * exporting a trained model
#
#   This class will automatically import an exported intprim model if it is found in the path specified by the "bip[x]/import_data" ROS parameter.
class IntPrimService():
    ##
    #   Initializes the IntPrimService class.
    #
    #   This 1) reads the parameters in /bip, 2) creates a corresponding BIP instance, and 3) spins up the ROS services.
    def __init__(self):
        rospy.init_node("intprim_service_node")

        self.bip_instances  = {}
        self.bip_parameters = {}
        self.bip_state      = {}
        self.last_generated_mean = None

        self.read_parameters()

        self.starting_time = None
        self.num_predictions = None
        self.playback_factor = 1.0

        self.primary_instance = None

        print("Creating Services")
        self.add_demo_service = rospy.Service(
            "/ip/addDemonstration",
            intprim_framework_ros.srv.AddDemonstration,
            self.add_demonstration_callback)

        self.compute_standardization_service = rospy.Service(
            "/ip/computeStandardization",
            intprim_framework_ros.srv.ComputeStandardization,
            self.compute_standardization_callback)

        self.eval_trajectory_service = rospy.Service(
            "/ip/evaluateTrajectory",
            intprim_framework_ros.srv.EvaluateTrajectory,
            self.evaluate_trajectory_callback)

        self.gen_trajectory_service = rospy.Service(
            "/ip/generateTrajectory",
            intprim_framework_ros.srv.GenerateTrajectory,
            self.generate_trajectory_callback)

        self.get_approximat_trajectory_service = rospy.Service(
            "/ip/getApproximateTrajectory",
            intprim_framework_ros.srv.GetApproximateTrajectory,
            self.get_approximate_trajectory_callback)

        self.get_distribution_service = rospy.Service(
            "/ip/getDistribution",
            intprim_framework_ros.srv.GetDistribution,
            self.get_probability_distribution_callback)

        self.get_distribution_parameters_service = rospy.Service(
            "/ip/getDistributionParameters",
            intprim_framework_ros.srv.GetDistributionParameters,
            self.get_distribution_parameters_callback)

        self.get_mean_trajectory_service = rospy.Service(
            "/ip/getMeanTrajectory",
            intprim_framework_ros.srv.GetMeanTrajectory,
            self.get_mean_trajectory_callback)

        self.get_statistics_service = rospy.Service(
            "/ip/getStatistics",
            intprim_framework_ros.srv.GetStatistics,
            self.get_statistics_callback)

        self.get_export_data_service = rospy.Service(
            "/ip/exportData",
            intprim_framework_ros.srv.ExportData,
            self.export_data_callback)

        self.get_initialize_state_service = rospy.Service(
            "/ip/initializeState",
            intprim_framework_ros.srv.InitializeState,
            self.initialize_state_callback)

        self.get_add_basis_selection_demo_service = rospy.Service(
            "/ip/addBasisSelectionDemonstration",
            intprim_framework_ros.srv.AddBasisSelectionDemonstration,
            self.add_basis_selection_demonstration_callback)

        self.get_perform_basis_selection_service = rospy.Service(
            "/ip/performBasisSelection",
            intprim_framework_ros.srv.PerformBasisSelection,
            self.perform_basis_selection_callback)

        self.get_basis_error_service = rospy.Service(
            "/ip/getBasisError",
            intprim_framework_ros.srv.GetBasisError,
            self.get_basis_error_callback)

        self.statistics_publisher = rospy.Publisher("/ip/statistics", intprim_framework_ros.msg.Statistics, queue_size = 1)

    ##
    # Creates a basis model for the given set of modality_params.
    #
    # @param modality_params The set of modality parameters as read from /bip[x]/modalities
    def create_model(self, modality_params):
        active_indices = modality_params["active"]

        # If active is True then it means all indices are active, so create an expanded list we can work with.
        if(modality_params["active"] == True):
            active_indices = [True] * len(modality_params["dof_names"])

        dof_names = list(itertools.compress(modality_params["dof_names"], active_indices))

        if(modality_params["basis_model"]["type"] == "Gaussian"):
            return bip.basis.gaussian_model.GaussianModel(modality_params["basis_model"]["degree"], modality_params["basis_model"]["scale"], dof_names, modality_params["basis_model"]["start_phase"], modality_params["basis_model"]["end_phase"])
        elif(modality_params["basis_model"]["type"] == "Sigmoidal"):
            return bip.basis.sigmoidal_model.SigmoidalModel(modality_params["basis_model"]["degree"], modality_params["basis_model"]["scale"], dof_names, modality_params["basis_model"]["start_phase"], modality_params["basis_model"]["end_phase"])
        elif(modality_params["basis_model"]["type"] == "Polynomial"):
            return bip.basis.polynomial_model.PolynomialModel(modality_params["basis_model"]["degree"], dof_names, modality_params["basis_model"]["start_phase"], modality_params["basis_model"]["end_phase"])

    ##
    # Reads the parameters from /bip[x] and calculates internal state information, such as which DoFs are active and which are used to generate a response trajectory.
    def read_parameters(self):
        for bip_params in rospy.get_param("bip"):
            dof_indices = []
            dof_names = []
            models = []
            generate_indices = []
            cov_noise = []
            active_dofs = []
            all_dof_names = []
            scaling_groups = {}

            cov_noise_buckets = set()

            current_dof_idx = 0
            for modality_params in bip_params["modalities"]:
                all_dof_names.extend(modality_params["dof_names"])

                # Active is either a single boolean value which applies to all DoFs or a list of booleans indicating whether each individual DoF is active
                if((isinstance(modality_params["active"], list) and True in modality_params["active"]) or modality_params["active"] == True):
                    active_indices = modality_params["active"]

                    # If active is True then it means all indices are active, so create an expanded list we can work with.
                    if(modality_params["active"] == True):
                        active_indices = [True] * len(modality_params["dof_names"])

                    num_active_indices = active_indices.count(True)

                    dof_indices.extend(list(itertools.compress(range(modality_params["indices"][0], modality_params["indices"][1]), active_indices)))

                    if(modality_params["generate"]):
                        generate_indices.extend(range(len(dof_names), len(dof_names) + num_active_indices))

                    # dof_names.extend(modality_params["dof_names"])

                    dof_names.extend(list(itertools.compress(modality_params["dof_names"], active_indices)))

                    models.append(self.create_model(modality_params))
                    # models[-1].plot()

                    # Map each DoF to its active phase duration.
                    # Will need to check against this in generate and whoever else calls localize.
                    # We have two options: a) check each DoF against a bounds to determine if the value gets set to high, or b) we create a "mask" to add to the cov matrix and which mask we use is dictated by our estimated phase.
                    cov_noise_buckets.add(modality_params["active_from"])
                    cov_noise_buckets.add(modality_params["active_until"])

                    # The scaling groups uses the "active" dof indices, rather than the original ones as these are given to BIP which does not have access to the full state list.
                    temp_idx = 0
                    for scale_idx, scale_group in enumerate(modality_params["scaling_groups"]):
                        if(active_indices[scale_idx]):
                            if(scale_group not in scaling_groups):
                                scaling_groups[scale_group] = []

                            scaling_groups[scale_group].append(current_dof_idx + temp_idx)
                            temp_idx += 1

                    current_dof_idx += num_active_indices

            cov_noise_buckets = sorted(cov_noise_buckets)
            all_active_dofs = []

            if(VERBOSE):
                print("Cov buckets" + str(cov_noise_buckets))

            for bucket_idx in range(len(cov_noise_buckets) - 1):
                bucket_start = cov_noise_buckets[bucket_idx]
                bucket_end = cov_noise_buckets[bucket_idx + 1]
                # For each bucket, append the limits of the bucket as well as a vector of zeros with the same dimensionality as the diagonal of the covariance matrix.
                cov_noise.append([[bucket_start, bucket_end], np.zeros(len(dof_names))])
                active_dofs.append([[bucket_start, bucket_end], []])

                current_dof_idx = 0
                for modality_params in bip_params["modalities"]:
                    if((isinstance(modality_params["active"], list) and True in modality_params["active"]) or modality_params["active"] == True):
                        active_indices = modality_params["active"]

                        # If active is True then it means all indices are active, so create an expanded list we can work with.
                        if(modality_params["active"] == True):
                            active_indices = [True] * len(modality_params["dof_names"])

                        num_active_indices = active_indices.count(True)

                        if(modality_params["active_from"] <= bucket_start and modality_params["active_from"] <= bucket_end and modality_params["active_until"] >= bucket_end and modality_params["generate"] is not True):
                            active_dofs[-1][1].extend(range(current_dof_idx, current_dof_idx + num_active_indices))
                            cov_noise[-1][1][current_dof_idx : current_dof_idx + num_active_indices] = modality_params["noise_bias"]
                        else:
                            cov_noise[-1][1][current_dof_idx : current_dof_idx + num_active_indices] = bip_params["filter"]["measurement_noise_bias"]

                        current_dof_idx += num_active_indices

                # Work around to add list items to set, as lists can't be added directly due to them being mutable.
                for item in active_dofs[-1][1]:
                    #all_active_dofs.add(item)
                    all_active_dofs.append(item)

                active_dofs[-1][1] = np.array(active_dofs[-1][1])

            self.bip_parameters[bip_params["id"]] = bip_params
            self.bip_parameters[bip_params["id"]]["dof_names"] = np.array(dof_names)
            self.bip_parameters[bip_params["id"]]["all_dof_names"] = np.array(all_dof_names)
            self.bip_parameters[bip_params["id"]]["dof_indices"] = np.array(dof_indices)
            self.bip_parameters[bip_params["id"]]["generate_indices"] = np.array(generate_indices)
            self.bip_parameters[bip_params["id"]]["cov_noise"] = cov_noise
            self.bip_parameters[bip_params["id"]]["cov_noise_buckets"] = np.array(cov_noise_buckets)
            self.bip_parameters[bip_params["id"]]["active_dofs"] = active_dofs
            self.bip_parameters[bip_params["id"]]["all_active_dofs"] = np.array(sorted(all_active_dofs))
            self.bip_parameters[bip_params["id"]]["scaling_groups"] = scaling_groups

            # print("Active DoFs: " + str(self.bip_parameters[bip_params["id"]]["active_dofs"]))
            if(VERBOSE):
                print("Active DoFs: " + str(active_dofs))
                print("All active DoFs: " + str(all_active_dofs))

            if(len(models) > 1):
                basis_model = bip.basis.mixture_model.MixtureModel(models)
            else:
                basis_model = models[0]

            scaling = None
            if(bip_params["scale_observations"]):
                scaling = scaling_groups.values()

            self.bip_instances[bip_params["id"]] = bip.BayesianInteractionPrimitive(basis_model, scaling)

            if("import_data" in bip_params):
                try:
                    self.bip_instances[bip_params["id"]].import_data(bip_params["import_data"])
                except Exception as e:
                    if(VERBOSE):
                        print("Failed to import data.")
                        print(e)
                    # traceback.print_exc()

            if(bip_params.get("primary", False)):
                self.primary_instance = bip_params["id"]

        # If we haven't found a primary primitive, set the first one as the primary.
        # This is for backwards compatibility when primary wasn't defined.
        if(self.primary_instance is None and len(self.bip_instances) > 0):
            self.primary_instance = self.bip_instances.keys()[0]

        self.playback_factor = rospy.get_param("interaction")["playback_factor"]
        self.use_spt = rospy.get_param("interaction")["single_point_trajectory"]["use"]
        self.spt_phase = rospy.get_param("interaction")["single_point_trajectory"]["phase"]
        self.start_generation_phase = rospy.get_param("interaction")["start_generation_phase"]
        self.stop_generation_phase = rospy.get_param("interaction")["stop_generation_phase"]
        self.response_frequency = rospy.get_param("interaction")["response_frequency"]

        self.basis_selection = None

    ##
    #   Initializes the current internal state.
    #   This includes the currently estimated phase state information as well as the current filter instance.
    def initialize_state(self):
        print("Resetting state")
        np.random.seed(23525234)

        for interaction_id, bip_params in self.bip_parameters.iteritems():
            self.bip_state[interaction_id] = {
                "current_phase": bip_params["filter"]["initial_phase"],
                "initial_phase": bip_params["filter"]["initial_phase"],
                "initial_phase_velocity": bip_params["filter"]["initial_phase_velocity"],
                "initial_phase_variance": bip_params["filter"]["initial_phase_variance"],
                "initial_phase_velocity_variance": bip_params["filter"]["initial_phase_velocity_variance"],
                "initial_phase_acceleration" : bip_params["filter"]["initial_phase_acceleration"],
                "initial_phase_acceleration_variance" : bip_params["filter"]["initial_phase_acceleration_variance"],
                "process_variance": bip_params["filter"]["process_variance"]
            }

            if(len(self.bip_instances[interaction_id].basis_weights) > 0):
                print("Initializing.")

                ensemble = None
                ensemble_size = bip_params["filter"]["ensemble_size"]

                if(bip_params["prior"]["init_with_demonstrations"] is True):
                    print("Num basis weights: " + str(len(self.bip_instances[interaction_id].basis_weights)))
                    if(bip_params["filter"]["ensemble_size"] < self.bip_instances[bip_params["id"]].basis_weights.shape[0]):
                        # We need to randomly sample some of the basis weights to fit the desired ensemble size
                        weights_to_use = np.random.choice(range(self.bip_instances[bip_params["id"]].basis_weights.shape[0]), bip_params["filter"]["ensemble_size"], replace = False)
                    else:
                        weights_to_use = np.array(range(self.bip_instances[bip_params["id"]].basis_weights.shape[0]))

                    ensemble = self.bip_instances[bip_params["id"]].basis_weights[weights_to_use, :]
                    ensemble_size = weights_to_use.shape[0]

                initial_phase_mean = [float(bip_params["filter"]["initial_phase"]), float(bip_params["filter"]["initial_phase_velocity"])]
                initial_phase_var = [float(bip_params["filter"]["initial_phase_variance"]), float(bip_params["filter"]["initial_phase_velocity_variance"])]

                if(bip_params["filter"]["name"] == "enkf"):
                    filter = intprim.filter.spatiotemporal.enkf.EnsembleKalmanFilter(
                        self.bip_instances[interaction_id].basis_model,
                        initial_phase_mean = initial_phase_mean,
                        initial_phase_var = initial_phase_var,
                        proc_var = float(bip_params["filter"]["process_variance"]),
                        time_delta = float(bip_params["filter"]["time_delta"]),
                        initial_ensemble = ensemble,
                        cyclical = bip_params["cyclical"])
                elif(bip_params["filter"]["name"] == "ekf"):
                    mean, cov = self.bip_instances[interaction_id].get_basis_weight_parameters()
                    filter = intprim.filter.spatiotemporal.ekf.ExtendedKalmanFilter(
                        self.bip_instances[interaction_id].basis_model,
                        initial_phase_mean = initial_phase_mean,
                        initial_phase_var = initial_phase_var,
                        proc_var = float(bip_params["filter"]["process_variance"]),
                        mean_basis_weights = mean,
                        cov_basis_weights = cov,
                        time_delta = float(bip_params["filter"]["time_delta"]),
                        cyclical = bip_params["cyclical"])
                elif(bip_params["filter"]["name"] == "pf"):
                    filter = intprim.filter.spatiotemporal.pf.ParticleFilter(
                        self.bip_instances[interaction_id].basis_model,
                        initial_phase_mean = initial_phase_mean,
                        initial_phase_var = initial_phase_var,
                        proc_var = float(bip_params["filter"]["process_variance"]),
                        time_delta = float(bip_params["filter"]["time_delta"]),
                        initial_ensemble = ensemble,
                        cyclical = bip_params["cyclical"])

                self.bip_instances[interaction_id].set_filter(filter)
            else:
                print("Skipping initialization.")

        self.starting_time = datetime.datetime.now()
        self.num_predictions = 0

    ##
    #   Callback for adding a demonstration.
    #   Simply passes the demonstration through to the desired BIP instance.
    #
    #   @param request Request containing the N x M observed demonstration trajectory, where N is the number of time steps and M is the number of features.
    #
    #   @returns True if demonstration was successfully added, false if not.
    def add_demonstration_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        # Request is rows = time steps, columns = features.
        # But BIP wants the transpose of that, so act accordingly.
        data = np.array(request.observed_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / request.observed_trajectory.stride, request.observed_trajectory.stride))
        data = data[:, self.bip_parameters[request.interaction_id]["dof_indices"]]
        self.bip_instances[request.interaction_id].add_demonstration(data.T)

        return intprim_framework_ros.srv.AddDemonstrationResponse(True)

    ##
    #   Callback for computing scaling parameters.
    #   @todo Need to update this call to use new scaling.
    def compute_standardization_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        # Request is rows = time steps, columns = features.
        # But BIP wants the transpose of that, so act accordingly.
        data = np.array(request.observed_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / request.observed_trajectory.stride, request.observed_trajectory.stride))
        data = data[:, self.bip_parameters[request.interaction_id]["dof_indices"]]
        self.bip_instances[request.interaction_id].compute_standardization(data.T)

        return intprim_framework_ros.srv.ComputeStandardizationResponse(True)

    ##
    #   Evaluates the given trajectory.
    #   Functionally, behaves the same as generate_trajectory_callback except the values corresponding to generate_indices need to be valid for the given trajectory.
    #   These values are used to compute the MSE between the estimated trajectory and the actual one.
    #   As in generate, this service can be called multiple times in order to calculate the error after a subset of observations have been made.
    #
    #   @param request Request containing a (possibly partial) N x M observed trajectory, where N is the number of time steps and M is the number of features.
    #                  Must also contain a M x M noise covariance matrix.
    #
    #   @returns float value which represents the mean squared error between the generated response trajectory and the actual response trajectory.
    def evaluate_trajectory_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        data = np.array(request.observed_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / request.observed_trajectory.stride, request.observed_trajectory.stride))
        data = data[:, self.bip_parameters[request.interaction_id]["dof_indices"]]
        covariance = self.get_covariance(request.covariance, data.shape[1], request.interaction_id)
        active_dofs = self.get_active_dofs(request.interaction_id)
        num_samples = int((1.0 - self.bip_state[request.interaction_id]["current_phase"]) * self.bip_parameters[request.interaction_id]["num_samples"])
        if(num_samples > 0):
            generated_trajectory, self.bip_state[request.interaction_id]["current_phase"], self.last_generated_mean, _ = self.bip_instances[request.interaction_id].generate_probable_trajectory_recursive(
                data.T,
                covariance,
                active_dofs,
                num_samples = num_samples,
                starting_phase = None,
                return_variance = False,
                phase_lookahead = self.bip_parameters[request.interaction_id]["phase_lookahead"])

            # Calculate MSE from the last observation compared to the first first generated trajectory sample.
            mse = sklearn.metrics.mean_squared_error(data[-1:, self.bip_parameters[request.interaction_id]["generate_indices"]], generated_trajectory[self.bip_parameters[request.interaction_id]["generate_indices"], :1].T)

            return intprim_framework_ros.srv.EvaluateTrajectoryResponse(
                mse
            )
        else:
            return intprim_framework_ros.srv.EvaluateTrajectoryResponse(
                0.0
            )

    ##
    #   Gets the covariance bucket index for the given BIP instance.
    #   A covariance "bucket" is created for each set of active phases, such that every phase value from [0, 1] belongs to a single active bucket.
    #   At run-time, we need to choose the appropriate covariance noise (some DoFs may be active and some inactive according to the parameter file), and so we need to find the proper bucket.
    #
    #   @param interaction_id The ID of the BIP instance to use.
    #
    #   @returns The index of the covariance bucket corresponding to the phase stored in current state of the given BIP instance.
    def get_bucket_index(self, interaction_id):
        bucket_idx = 0
        for bucket_limits, cov_noise in self.bip_parameters[interaction_id]["cov_noise"]:
            # print("Bucket limits: " + str(bucket_limits))
            if(self.bip_state[interaction_id]["current_phase"] >= bucket_limits[0] and self.bip_state[interaction_id]["current_phase"] < bucket_limits[1]):
                return bucket_idx
            bucket_idx += 1

    ##
    #   Gets the covariance matrix for the given BIP instance and measurement noise.
    #   This covariance matrix accounts for both the given measurement noise as well as any active/inactive DoFs corresponding to the currently estimated phase.
    #   Active DoFs will have a noise value equivalent to the "noise_bias" parameter, inactive DoFs to "inactive_noise_bias".
    #
    #   @param covariance 2M x 1 measurement noise vector, where M is the number of features.
    #   @param stride The stride of the measurement noise vector: M
    #   @param interaction_id The ID of the BIP instance to use.
    #
    #   @returns The M x M covariance noise matrix corresponding to the currently estimated phase for the given BIP instance.
    def get_covariance(self, covariance, stride, interaction_id):
        covariance = np.array(covariance, dtype = np.float64)
        covariance = np.reshape(covariance, (stride, stride))

        bucket_idx = self.get_bucket_index(interaction_id)

        if(bucket_idx is not None):
            covariance[np.diag_indices(covariance.shape[0])] += self.bip_parameters[interaction_id]["cov_noise"][bucket_idx][1]
            return covariance

    ##
    #   Gets the list of DoFs that are active according to the current state of the given BIP instance.
    #
    #   @param interaction_id The ID of the BIP instance to use.
    #
    #   @returns N dimensional vector containing indices into the full state dimension, where N is the number of currently active DoFs.
    def get_active_dofs(self, interaction_id):
        bucket_idx = self.get_bucket_index(interaction_id)

        if(bucket_idx is not None):
            return self.bip_parameters[interaction_id]["active_dofs"][bucket_idx][1]

    ##
    #   Performs inference on the given partially observed trajectory using the given BIP instance.
    #   This generation is recursive, and so the current state will be used (and updated) in the integration of any new sensor measurements.
    #
    #   @param request Request containing a (possibly partial) N x M observed trajectory, where N is the number of time steps and M is the number of features.
    #                  Must also contain a M x M noise covariance matrix.
    #
    #   @returns P x Q response trajectory, where P is the number of time steps until the end of the demonstration and Q is the number of generated features.
    def generate_trajectory_callback(self, request):
        self.num_predictions += 1

        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        data    = np.array(request.observed_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / request.observed_trajectory.stride, request.observed_trajectory.stride))
        data = data[:, self.bip_parameters[request.interaction_id]["dof_indices"]]
        covariance = self.get_covariance(request.covariance, data.shape[1], request.interaction_id)
        active_dofs = self.get_active_dofs(request.interaction_id)

        if(self.use_spt is True):
            num_samples = 1
            if(self.spt_phase == "current"):
                starting_phase = None
            else:
                starting_phase = self.spt_phase
        else:
            num_samples = int((1.0 - self.bip_state[request.interaction_id]["current_phase"]) * self.bip_parameters[request.interaction_id]["num_samples"])
            starting_phase = None

        if(num_samples == 0):
            num_samples = 1

        if(active_dofs is not None):
            generated_trajectory, self.bip_state[request.interaction_id]["current_phase"], self.last_generated_mean, _ = self.bip_instances[request.interaction_id].generate_probable_trajectory_recursive(
                data.T,
                covariance,
                active_dofs,
                num_samples = num_samples,
                starting_phase = starting_phase,
                return_variance = False,
                phase_lookahead = self.bip_parameters[request.interaction_id]["phase_lookahead"])

            print("Phase: " + str(self.bip_state[request.interaction_id]["current_phase"]))
            #print("Num samples: " + str(num_samples))
            print("Observed trajectory size: " + str(data.shape[0]))
            #print("Stride: " + str(data.shape[1]))

            if(self.bip_parameters[request.interaction_id]["debug"]):
                self.stat_collector.collect(self.bip_instances[request.interaction_id], data, generated_trajectory.T, rospy.get_time())

            if(self.bip_state[request.interaction_id]["current_phase"] > self.stop_generation_phase or self.bip_state[request.interaction_id]["current_phase"] < self.start_generation_phase):
                print("Sending empty trajectory...")

                return intprim_framework_ros.srv.GenerateTrajectoryResponse(
                    intprim_framework_ros.msg.Trajectory(len(self.bip_parameters[request.interaction_id]["generate_indices"]), [])
                )
            else:
                return intprim_framework_ros.srv.GenerateTrajectoryResponse(
                    intprim_framework_ros.msg.Trajectory(len(self.bip_parameters[request.interaction_id]["generate_indices"]), generated_trajectory[self.bip_parameters[request.interaction_id]["generate_indices"]].T.flatten().tolist())
                )
        else:
            print("Sending empty trajectory...")
            return intprim_framework_ros.srv.GenerateTrajectoryResponse(
                intprim_framework_ros.msg.Trajectory(len(self.bip_parameters[request.interaction_id]["generate_indices"]), [])
            )

    ##
    #   Gets the basis approximation for the given trajectory and BIP instance.
    #
    #   @param request Request containing an N x M observed trajectory, where N is the number of time steps and M is the number of features.
    #
    #   @returns P x M approximated trajectory, where P is the number of time steps specified in the request and M is the number of features.
    def get_approximate_trajectory_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        data = np.array(request.observed_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / request.observed_trajectory.stride, request.observed_trajectory.stride))
        data = data[:, self.bip_parameters[request.interaction_id]["dof_indices"]]

        approx_traj = self.bip_instances[request.interaction_id].get_approximate_trajectory(data.T, num_samples = request.num_samples)

        return intprim_framework_ros.srv.GetApproximateTrajectoryResponse(
            intprim_framework_ros.msg.Trajectory(approx_traj.shape[0], approx_traj.T.flatten().tolist())
        )

    ##
    #   Gets the probability distribution of the prior for the given BIP instance.
    #   The distribution is assumed to be Gaussian from which the parameters are found empirically.
    #
    #   @param request Request containing the ID of the BIP instance to use.
    #
    #   @returns three N x M matrices representing the mean, upper bound, and lower bound (within one standard deviation) where N is the number of time steps and M is the number of features.
    def get_probability_distribution_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        mean, upper_bound, lower_bound = self.bip_instances[request.interaction_id].get_probability_distribution()

        return intprim_framework_ros.srv.GetDistributionResponse(
            intprim_framework_ros.msg.Trajectory(mean.shape[0], mean.T.flatten().tolist()),
            intprim_framework_ros.msg.Trajectory(upper_bound.shape[0], upper_bound.T.flatten().tolist()),
            intprim_framework_ros.msg.Trajectory(lower_bound.shape[0], lower_bound.T.flatten().tolist())
        )

    ##
    #   Gets the parameters associated with the probability distribution of the prior for the given BIP instance.
    #   The distribution is assumed to be Gaussian from which the parameters are found empirically.
    #
    #   @param request Request containing the ID of the BIP instance to use.
    #
    #   @returns M x 1 vector containing the mean and a M x M matrix containing the covariance.
    def get_distribution_parameters_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        mean, var = self.bip_instances[request.interaction_id].get_basis_weight_parameters()
        var = np.array(self.bip_instances[request.interaction_id].basis_weights, dtype = np.float64)
        var = np.corrcoef(var.T)

        return intprim_framework_ros.srv.GetDistributionParametersResponse(
            var.flatten().tolist(),
            mean.tolist()
        )

    ##
    #   Gets the mean trajectory of the prior for the given BIP instance.
    #
    #   @param request Request containing the ID of the BIP instance to use.
    #
    #   @returns N x M trajectory containing the mean of the trained prior, where N is the number of time steps and M is the number of features.
    def get_mean_trajectory_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        generated_trajectory = self.bip_instances[request.interaction_id].get_mean_trajectory(request.num_samples)

        if(request.return_full_trajectory):
            return intprim_framework_ros.srv.GetMeanTrajectoryResponse(
                intprim_framework_ros.msg.Trajectory(generated_trajectory.shape[0], generated_trajectory.T.flatten().tolist())
            )
        else:
            return intprim_framework_ros.srv.GetMeanTrajectoryResponse(
                intprim_framework_ros.msg.Trajectory(len(self.bip_parameters[request.interaction_id]["generate_indices"]), generated_trajectory[self.bip_parameters[request.interaction_id]["generate_indices"]].T.flatten().tolist())
            )

    ##
    #   Gets the analysis statistics associated with the current state.
    #   By default this includes information such as the number of predictions, their frequency, and the time elapsed since initialize was called.
    #   If the debug ROS parameter is set to true, then the internal filter state for each time step is written to an XML file.
    #
    #   Note: depending on the response frequency and state dimension this XML file can be quite large and may take some time to write do disk.
    #
    #   @param request Request containing the ID of the BIP instance to analyze. If a ground truth rosbag file path is specified, its information will be included in the exported debug XML file.
    #
    #   @returns The default analysis statistics.
    def get_statistics_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        ending_time = datetime.datetime.now()
        duration = (ending_time - self.starting_time).total_seconds()
        frequency = float(self.num_predictions) / (float(duration) * self.playback_factor)

        message = intprim_framework_ros.msg.Statistics(self.num_predictions, duration, frequency)

        # Publish for easy debugging
        self.statistics_publisher.publish(message)

        # Export debugging XML file here.
        if(self.bip_parameters[request.interaction_id]["debug"]):
            self.stat_collector.export(self.bip_instances[request.interaction_id], self.bip_parameters[request.interaction_id]["debug_directory"], request.bag_file, self.bip_parameters[request.interaction_id]["num_samples"])

        # Return values as part of service call as well
        return intprim_framework_ros.srv.GetStatisticsResponse(message)

    ##
    #   Exports the given BIP instance. This includes the trained prior and other internal state information.
    #
    #   @param request Request containing the ID of the BIP instance to export as well as the file name that it should be exported to.
    #
    #   @returns True if export was successful, false otherwise.
    def export_data_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        self.bip_instances[request.interaction_id].export_data(request.file_name)

        return intprim_framework_ros.srv.ExportDataResponse(True)

    ##
    #   Initializes the internal state of the given BIP instance.
    #   This resets the filter back to initial starting conditions.
    #
    #   @param request Request containing the ID of the BIP instance to initialize.
    #
    #   @returns True if initialization was successful, false if not.
    def initialize_state_callback(self, request):
        self.read_parameters()

        self.initialize_state()

        # Initialize stat collection for debugging
        if(self.bip_parameters[self.primary_instance]["debug"]):
            self.stat_collector = analysis.stat_collector.StatCollector(self.bip_instances[self.primary_instance], self.bip_parameters[self.primary_instance]["generate_indices"], np.setdiff1d(self.bip_parameters[self.primary_instance]["all_active_dofs"], self.bip_parameters[self.primary_instance]["generate_indices"]))

            generated_trajectory = self.bip_instances[self.primary_instance].get_mean_trajectory(num_samples = self.bip_parameters[self.primary_instance]["num_samples"])
            self.stat_collector.collect(self.bip_instances[self.primary_instance], np.array([[] for _ in range(generated_trajectory.shape[0])]), generated_trajectory.T, None)

        return intprim_framework_ros.srv.InitializeStateResponse(True)

    ##
    #   Adds a demonstration to a basis Selection instance.
    #   Note: demonstrations must first be added via add_basis_selection_demonstration_callback.
    #
    #   @param request Request containing the N x M observed demonstration trajectory, where N is the number of time steps and M is the number of features.
    #
    #   @returns True if demonstration was sucessfully added, false if not.
    def add_basis_selection_demonstration_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        # Request is rows = time steps, columns = features.
        # But BIP wants the transpose of that, so act accordingly.
        data = np.array(request.observed_trajectory.data, dtype = np.float64)
        data = np.reshape(data, (len(data) / request.observed_trajectory.stride, request.observed_trajectory.stride))
        data = data[:, self.bip_parameters[request.interaction_id]["dof_indices"]]

        if(self.basis_selection is None):
            scaling = None
            if(self.bip_parameters[request.interaction_id]["scale_observations"]):
                scaling = self.bip_parameters[request.interaction_id]["scaling_groups"].values()

            self.basis_selection = bip.basis.selection.Selection(self.bip_parameters[request.interaction_id]["dof_names"], scaling)

        self.basis_selection.add_demonstration(data.T)

        return intprim_framework_ros.srv.AddBasisSelectionDemonstrationResponse(True)

    ##
    #   Selects an appropriate basis space for the demonstrations which have been added to the Selection instance.
    #
    #   @param request Request containing the ID of the BIP instance to perform basis selection with.
    #
    #   @returns True if basis selection was successfully performed, false otherwise.
    def perform_basis_selection_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        start_dof_idx = 0
        for modality_params in self.bip_parameters[request.interaction_id]["modalities"]:
            if(modality_params["active"]):
                end_dof_idx = start_dof_idx + (modality_params["indices"][1] - modality_params["indices"][0])

                aic, bic = self.basis_selection.get_information_criteria(np.arange(start_dof_idx, end_dof_idx), modality_params["basis_model"]["start_phase"], modality_params["basis_model"]["end_phase"], modality_params["active_from"], modality_params["active_until"])

                print("Best model for modality " + str(modality_params["name"]))
                aic_model, bic_model = self.basis_selection.get_best_model(aic, bic)

                start_dof_idx = end_dof_idx
            else:
                print("Modality " + str(modality_params["name"]) + " is not active. Skipping...")

        self.basis_selection = None

        return intprim_framework_ros.srv.PerformBasisSelectionResponse(True)

    ##
    #   Calculates the MSE error for the basis space of the given BIP instance for the demonstrations which have been added to the Selection instance.
    #   Note: demonstrations must first be added via add_basis_selection_demonstration_callback.
    #
    #   @param request Request containing the ID of the BIP instance to perform basis selection with.
    #
    #   @returns M x M matrix containing the MSE errors along the diagonal.
    #   @todo Update so that only the diagonal is returned.
    def get_basis_error_callback(self, request):
        if(request.interaction_id == -1):
            request.interaction_id = self.primary_instance

        errors = None
        start_dof_idx = 0
        for modality_params in self.bip_parameters[request.interaction_id]["modalities"]:
            if(modality_params["active"]):
                end_dof_idx = start_dof_idx + (modality_params["indices"][1] - modality_params["indices"][0])

                model = self.create_model(modality_params)
                mse = self.basis_selection.get_model_mse(model, np.arange(start_dof_idx, end_dof_idx), modality_params["active_from"], modality_params["active_until"])

                if(errors is None):
                    errors = mse
                else:
                    errors = np.hstack((errors, mse))

                start_dof_idx = end_dof_idx
            else:
                print("Modality " + str(modality_params["name"]) + " is not active. Skipping...")

        self.basis_selection = None

        observation_noise = np.diag(errors)

        return intprim_framework_ros.srv.GetBasisErrorResponse(observation_noise.flatten(), errors.shape[0])

    def run(self):
        rospy.spin()

## @}

if __name__ == '__main__':
    service = IntPrimService()

    service.run()
