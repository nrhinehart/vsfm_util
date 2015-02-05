from collections import OrderedDict
menu = OrderedDict()

file_menu = OrderedDict({33166	: 'open_multi_images',
             33028	: 'open_image_and_sift',
             33186	: 'open_current_path',
             32928	: 'detect_features',
             33167	: 'load_feature_file',
             32841	: 'new_window',
             105	: 'close_window', 
             32842	: 'exit_program'})


sfm_menu = OrderedDict()
sfm_twoview_menu = OrderedDict({33046  : 'two_view_match',
                    33018  : 'feature_match',
                    33023  : 'f_matrix_match',
                    33277  : 'guided_match',
                    33059  : 'h_matrix_match',
                    33057  : 'save_inlier_match',
                    33021  : 'load_inlier_match',
                    33239  : 'discard_inlier_match',
                    33282  : 'save_as_nv_match',
                    33296  : 'mutual_best_match',
                    33298  : 'use_small_features',
                    33499  : 'no_stationary_points'})


sfm_pairwise_menu = OrderedDict({33033 : 'compute_missing_match',
                     33487 : 'compute_specified_match',
                     33498 : 'compute_sequence_match',
                     33043 : 'compute_missing_f_matrix',
                     33220 : 'update_pairwise_f_matrix',
                     33507 : 'use_preemptive_matching',
                     33191 : 'multi_threaded_match',
                     33477 : 'asynchronous_match',
                     33228 :  'use_filetitle_as_identifier',
                     33486 : 'import_feature_matches',
                     33473 : 'export_feature_matches',
                     33503 : 'export_f_matrix_matches',
                     34000 : 'show_spanning_forest',
                     33268 : 'show_match_matrix'})

sfm_more_menu = OrderedDict({33152  : 'reload_all_settings',
            33212  : 'start_new_model',
            33184  : 'set_initialization_pair',
            33299  : 'set_fixed_calibration',
            33061  : 'bundle_adjustment',
            33523  : 'reconstruct_mesh',
            33489  : 'gcp_based_transform',
            33500  : 'gps_based_transform',
            33066  : 'find_more_points',
            33531  : 'run_constrained_ba',
            33367  : 'update_point_color',
            33405  : 'update_thumbnails',
            33508  : 'use_shared_calibration',
            33210  : 'search_multiple_models',
            33185  : 'use_radial_distortion',
            33201  : 'filter_unstable_points',
            33229  : 'less_visualization_data',
            33519  : 'use_level_0_for_pmvs'})

sfm_extra_menu = OrderedDict({33225 : 'model_information',
                  33200	: 'save_compact_nvm',
                  33223	: 'save_selected_model',
                  33226	: 'save_separate_models',
                  33048	: 'delete_current_photo',
                  33198	: 'delete_reconstructed',
                  33355	: 'delete_not_in_nvm'})
                  
sfm_menu['menu_sfm_twoview'] = sfm_twoview_menu
sfm_menu['menu_sfm_pairwise'] = sfm_pairwise_menu
sfm_menu['menu_sfm_more'] = sfm_more_menu
sfm_menu['menu_sfm_extra'] = sfm_extra_menu
sfm_menu[33041] = 'reconstruct_sparse'
sfm_menu[33065] = 'reconstruct_resume'
sfm_menu[33471] = 'reconstruct_dense'
sfm_menu[33045] = 'load_nview_match'
sfm_menu[33202] = 'add_nview_match'
sfm_menu[33044]	= 'save_nview_match'
sfm_menu[33047]	= 'clear_workspace'
sfm_menu[33074]	= 'delete_selected_camera'
sfm_menu[33216]	= 'delete_selected_model'
sfm_menu[33237]	= 'delete_all_models'


view_menu = OrderedDict({32777 : 'single_image',
                         33019 : 'feature_matches',
                         33007 : 'inlier_matches',
                         33005 : '2_view_3d_points',
                         33037 : 'n_view_3d_points',
                         33467 : 'dense_3d_points',
                         33190 : 'image_thumbnails',
                         33530 : 'perspective_view',
                         33451 : 'dark_background',
                         33218 : 'show_single_model',
                         33034 : 'show_2view_tracks',
                         33078 : 'hilight_image',
                         33038 : 'hilight_matcher',
                         33032 : 'next_photo_pair',
                         33049 : 'prev_photo_pair'})
                        
view_options_menu = OrderedDict({33077 : 'switch_2d_3d',
                                 33070 : 'show_3+_points',
                                 33042 : 'textured_camera',
                                 33012 : 'show_bounding_box',
                                 33505 : 'downward_3d_y_axis',
                                 32922 : 'show_features',
                                 33234 : 'tight_thumbnails',
                                 33025 : 'show_rand_match',
                                 33344 : 'horizontal_layout',
                                 33246 : 'align_two_images'})
view_menu['menu_view_options'] = view_options_menu                      

menu['menu_file'] = file_menu
menu['menu_sfm'] = sfm_menu
menu['menu_view'] = view_menu
