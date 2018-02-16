
# flag to switch to custom params defined in this message. If not set, GraspIt's default settings are kept
bool set_custom_params


# //Annealing parameters
# //! Annealing constant for neighbor generation schedule
float64 YC	 	#	GraspIt! default: 7.0
# //! Annealing constant for error acceptance schedule
float64 HC	 	#	GraspIt! default: 7.0
# //! Number of dimensions for neighbor generation schedule
float64 YDIMS	#	GraspIt! default: 8.0
# //! Number of dimensions for error acceptance schedule
float64 HDIMS	#	GraspIt! default: 8.0
# //! Adjust factor for neighbor generation schedule
float64 NBR_ADJ	#	GraspIt! default: 1.0
# //! Adjust raw errors reported by states to be in the relevant range of the annealing schedule
float64 ERR_ADJ	#	GraspIt! default: 1.0e-6
# //! Starting temperatue
float64 DEF_T0	#	GraspIt! default: 1e6
# //! Starting step
float64 DEF_K0	#	GraspIt! default: 30000