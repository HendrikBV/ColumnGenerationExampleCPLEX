\ENCODING=ISO-8859-1
\Problem name: master_problem

Minimize
 obj: 10000 z_super_1 + 10000 z_super_2 + 10000 z_super_3 + 10 z_1_1 + 5 z_3_6
      + 20 z_2_8 + 15 z_3_9 + 15 z_1_10
Subject To
 Lecture_1_assigned_to_exactly_one_timeslot#0: z_super_1 + z_1_1 + z_2_2
                                 + z_3_3 + z_3_15  = 1
 Lecture_2_assigned_to_exactly_one_timeslot#1: z_super_1 + z_1_4 + z_2_5
                                 + z_3_6 + z_1_13 + z_2_14  = 1
 Lecture_3_assigned_to_exactly_one_timeslot#2: z_super_1 + z_1_7 + z_2_8
                                 + z_3_9 + z_1_13  = 1
 Lecture_4_assigned_to_exactly_one_timeslot#3: z_super_1 + z_1_10 + z_2_11
                                 + z_3_12 + z_2_14 + z_3_15  = 1
 _Exactly_one_column_selected_for_timeslot_1#4: z_super_1 + z_1_1 + z_1_4
                                 + z_1_7 + z_1_10 + z_1_13  = 1
 _Exactly_one_column_selected_for_timeslot_2#5: z_super_2 + z_2_2 + z_2_5
                                 + z_2_8 + z_2_11 + z_2_14  = 1
 _Exactly_one_column_selected_for_timeslot_3#6: z_super_3 + z_3_3 + z_3_6
                                 + z_3_9 + z_3_12 + z_3_15  = 1
Bounds
 0 <= z_super_1 <= 1
 0 <= z_super_2 <= 1
 0 <= z_super_3 <= 1
 0 <= z_1_1 <= 1
 0 <= z_2_2 <= 1
 0 <= z_3_3 <= 1
 0 <= z_1_4 <= 1
 0 <= z_2_5 <= 1
 0 <= z_3_6 <= 1
 0 <= z_1_7 <= 1
 0 <= z_2_8 <= 1
 0 <= z_3_9 <= 1
 0 <= z_1_10 <= 1
 0 <= z_2_11 <= 1
 0 <= z_3_12 <= 1
 0 <= z_1_13 <= 1
 0 <= z_2_14 <= 1
 0 <= z_3_15 <= 1
End
