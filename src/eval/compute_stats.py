
import numpy as np
import trajectory_planning_helpers as tph


def compute_stats(result_file):
    # Load the CSV with the optimized trajectory
    results = np.loadtxt(result_file, delimiter=",")

    # Compute distances between trajectory points
    distances = np.zeros((len(results), 1))
    distance_vectors = np.zeros((len(results), 2))
    distance_vectors[0:-2, :] = results[1:-1, :] - results[0:-2, :]
    distance_vectors[-2, :] = results[-2, :] - results[-1, :]
    distance_vectors[-1, :] = results[-1, :] - results[0, :]
    distances[:, 0] = np.linalg.norm(distance_vectors, axis=1)

    #print(distances)

    # Compute curvatures
    headings, curvatures = tph.calc_head_curv_num.calc_head_curv_num(
        path=results,
        el_lengths=distances,
        is_closed=True
    )
    total_curvature = np.sum(curvatures)

    # Import vehicle dynamics
    ggv, ax_max_machines = tph.import_veh_dyn_info.import_veh_dyn_info(
        ggv_import_path="eval/veh_dyn_info/ggv.csv",
        ax_max_machines_import_path="eval/veh_dyn_info/ax_max_machines.csv"
    )

    # Compute velocity and acceleration profile
    vel_profile = tph.calc_vel_profile.calc_vel_profile(
        ggv=ggv,
        ax_max_machines=ax_max_machines,
        v_max=70.0,
        kappa=curvatures,
        el_lengths=distances,
        closed=True,
        filt_window=None,
        dyn_model_exp=1.0,
        drag_coeff=0.75,
        m_veh=1200.0
    )

    vel_profile_loop = np.append(vel_profile, vel_profile[0])
    acc_profile = tph.calc_ax_profile.calc_ax_profile(
        vx_profile=vel_profile_loop,
        el_lengths=distances,
        eq_length_output=False
    )[0]

    # Calculate laptime
    laptime = tph.calc_t_profile.calc_t_profile(
        vx_profile=vel_profile,
        ax_profile=acc_profile,
        el_lengths=distances
    )[-1]

    return total_curvature, laptime, vel_profile, acc_profile


if __name__ == "__main__":
    results_centerline = compute_stats("output/centerline.csv")
    results_optimization_based = compute_stats("output/optimization_based.csv")
    results_pi = compute_stats("output/pi_results.csv")

    print("|                 Results | Lap Time |")
    print("| ----------------------- | -------- |")
    print("|              Centerline | %.2fs   |" % results_centerline[1])
    print("|      Optimization-based | %.2fs   |" % results_optimization_based[1])
    print("| Probabilistic Inference | %.2fs   |" % results_pi[1])
    
    np.savetxt("output/pi_results_vel_profile.csv", results_pi[2], delimiter="\n")
    np.savetxt("output/pi_results_acc_profile.csv", results_pi[3], delimiter="\n")

