/* ==================================================================
                               INCLUDES
   ================================================================== */

#include "ilcplex\cplex.h"		// include files for CPLEX

#include <vector>				// so we can use a vector
#include <chrono>				// so we can use timers
#include <stdexcept>			// so we can throw and catch exceptions
#include <fstream>				// to enable input from files and output to files
#include <iostream>				// to write output to the screen
#include <memory>				// so we can use unique_ptr for arrays
#include <string>				// so we can use strings





/* ==================================================================
				             GLOBAL VARIABLES
   ================================================================== */

// ================ PROBLEM DATA ================ 

// The number of lectures
int nb_lectures;

// The number of timeslots
int nb_timeslots;

// The number of rooms
int nb_rooms;

// The number of professors
int nb_professors;

// Indicates which professors teach which lectures
std::vector<int> professors_lecture;	// professor_lecture.at(p * nb_lectures + l) == 1 if professor p teacher lecture l, 0 otherwise

// Indicates the cost of assigning a lecture to a given timeslot
std::vector<int> cost_lecture_timeslot; // cost_lecture_timeslot.at(l * nb_timeslots + t) == cost of assigning lecture l to timeslot t



// ================ CPLEX VARIABLES ================ 

// A pointer to the CPLEX environment
CPXENVptr env = nullptr;

// A pointer to the master problem
CPXLPptr master_problem = nullptr;

// A pointer to the pricing problem
CPXLPptr pricing_problem = nullptr;

// Char buffer for CPLEX error messages
char error_text[CPXMESSAGEBUFSIZE];

// The solution of the master problem from CPLEX
std::unique_ptr<double[]> solution_masterproblem;

// Indices of the variables for which we change the coefficients in the pricing problem
std::unique_ptr<int[]> indices_variables_pricingproblem;

// Coefficients of the variables for which we change the coefficients in the pricing problem
std::unique_ptr<double[]> new_coefficients_pricingproblem;

// The dual prices from the master problem
std::unique_ptr<double[]> dual_prices;

// Objective value of the master problem
double objective_value_masterproblem;

// Objective value of the pricing problem
double objective_value_pricingproblem;

// Objective function coefficient in master of newly found column
double cost_new_column;

// Values in constraint matrix master of newly found column
std::vector<int> new_column_a;

// The number of columns added
int nb_columns_added;

// The computation time in seconds
double computation_time;





/* ==================================================================
						READ PROBLEM DATA
   ================================================================== */

void read_problem_data_from_file(const std::string& file_name)
{
	std::ifstream file;
	file.open(file_name);
	if (!file.is_open())
		throw std::runtime_error("Couldn't open the input file name \"" + file_name + "\"");

	// Read in the basic data
	file >> nb_lectures;
	file >> nb_timeslots;
	file >> nb_rooms;
	file >> nb_professors;

	// Read in the professor_lecture matrix
	for (int p = 0; p < nb_professors; ++p)
	{
		for (int l = 0; l < nb_lectures; ++l)
		{
			int k;
			file >> k;
			professors_lecture.push_back(k);
		}
	}

	// Read in the cost_lecture_timeslot matrix
	for (int l = 0; l < nb_lectures; ++l)
	{
		for (int t = 0; t < nb_timeslots; ++t)
		{
			int k;
			file >> k;
			cost_lecture_timeslot.push_back(k);
		}
	}
}





/* ==================================================================
	                        INITIALIZE CPLEX
   ================================================================== */

void initialize_cplex()
{
	int status = 0;

	// Open the cplex environment
	env = CPXopenCPLEX(&status);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function initialize_cplex(). \nCouldn't open CPLEX. \nReason: " + std::string(error_text));
	}

	// Set the output to screen on/off
	status = CPXsetintparam(env, CPX_PARAM_SCRIND, CPX_OFF);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function initialize_cplex(). \nCouldn't set the output to screen off. \nReason: " + std::string(error_text));
	}
}




/* ==================================================================
                           BUILD MASTER PROBLEM
   ================================================================== */

void build_master_problem()
{
	int status = 0;
	double obj[1];						// Objective function coefficient of variables
	double lb[1];						// Lower bounds of variables
	double ub[1];						// Upper bounds of variables
	double rhs[1];						// Right-hand side of constraints
	char *colname[1];					// Variable names
	char *rowname[1];					// Constraint names
	char sense[1];						// Sign of constraint ('L' for <=, 'E' for ==, 'G' for >=)
	int matbeg[1];						// Begin position of the constraint (always 0)
	std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
	std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
	int nonzeros = 0;					// To calculate number of nonzero coefficients in each constraint


	// Create arrays
	matind = std::make_unique<int[]>(2000);
	matval = std::make_unique<double[]>(2000);

	// Create the problem
	master_problem = CPXcreateprob(env, &status, "master_problem");
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function build_master_problem(). \nCouldn't create the CPLEX problem. \nReason: " + std::string(error_text));
	}

	// Problem is minimization
	CPXchgobjsen(env, master_problem, CPX_MIN);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function build_master_problem(). \nCouldn't set the problem type to minimization. \nReason: " + std::string(error_text));
	}

	// VARIABLES
	// Add the z_tj variables
	// Here we add a super column for every timeslot
	for(int t = 0; t < nb_timeslots; ++t)
	{
		std::string name = "z_super_" + std::to_string(t + 1);
		colname[0] = const_cast<char*>(name.c_str());

		obj[0] = 10000; // cost of the super column: very high objective function coefficient

		lb[0] = 0;
		ub[0] = 1;

		status = CPXnewcols(env, master_problem, 1, obj, lb, ub, NULL, colname); // Generate columns (the variables) 
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function build_master_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
		}
	}

	// CONSTRAINTS
	// Constraint set 1: every lecture assigned to exactly one timeslot
	for (int l = 0; l < nb_lectures; ++l) // forall(l in L)
	{
		sense[0] = 'E';	// constraint is '=='
		rhs[0] = 1;		// right-hand side of constraint equals 1

		std::string name = "Lecture_" + std::to_string(l + 1) + "_assigned_to_exactly_one_timeslot";
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;	// always 0
		nonzeros = 0;	// number of coefficients in constraint different from 0

		for (int t = 0; t < nb_timeslots; ++t) // sum(t in T)
		{
			if (t == 0) // the super column for the first timeslot contains all lectures
			{
				matind[nonzeros] = t;	// z_super_t is at position t in the constraint matrix
				matval[nonzeros] = 1;	// z_super_0 has a coefficient of 1 in this constraint
				++nonzeros;		// increase variable nonzeros by 1, as we have a variable with nonzero coefficient in the constraint
			}
			else
			{
				// do nothing
				// other super columns contain no lectures
			}
		}

		// add the constraint to CPLEX
		status = CPXaddrows(env, master_problem, 0, 1, nonzeros, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function build_master_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}
	}

	// Constraint set 2: exactly one column selected for every timeslot
	for (int t = 0; t < nb_timeslots; ++t) // forall(t in T)
	{
		sense[0] = 'E';	// constraint is '=='
		rhs[0] = 1;		// right-hand side of constraint equals 1

		std::string name = "Exactly_one_column_selected_for_timeslot_" + std::to_string(t + 1);
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;	// always 0
		nonzeros = 0;	// number of coefficients in constraint different from 0

		matind[nonzeros] = t;	// z_super_t is at position t in the constraint matrix
		matval[nonzeros] = 1;	// z_super_0 has a coefficient of 1 in this constraint
		++nonzeros;		// increase variable nonzeros by 1, as we have a variable with nonzero coefficient in the constraint

		// add the constraint to CPLEX
		status = CPXaddrows(env, master_problem, 0, 1, nonzeros, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function build_master_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}
	}


	// Write the problem to an LP-file, so that we can check whether it is correct
	status = CPXwriteprob(env, master_problem, "master_problem.lp", NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function build_master_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
	}


	// Allocate memory for dual variables
	indices_variables_pricingproblem = std::make_unique<int[]>(nb_lectures*nb_rooms);

	for (int i = 0; i < nb_lectures*nb_rooms; ++i)
		indices_variables_pricingproblem[i] = i;

	new_coefficients_pricingproblem = std::make_unique<double[]>(nb_lectures*nb_rooms);

	dual_prices = std::make_unique<double[]>(nb_lectures + nb_timeslots);

	new_column_a.reserve(nb_lectures);
	for (int l = 0; l < nb_lectures; ++l)
		new_column_a.push_back(0);
}





/* ==================================================================
						 BUILD PRICING PROBLEM
   ================================================================== */

// We build a single pricing problem instead of one pricing problem for all timeslots.
// We will change the relevant coefficients each time we solve the pricing problem 
// for a different timeslot.
void build_pricing_problem()
{
	int status = 0;
	double obj[1];						// Objective function coefficient of variables
	double lb[1];						// Lower bounds of variables
	double ub[1];						// Upper bounds of variables
	double rhs[1];						// Right-hand side of constraints
	char *colname[1];					// Variable names
	char *rowname[1];					// Constraint names
	char sense[1];						// Sign of constraint ('L' for <=, 'E' for ==, 'G' for >=)
	int matbeg[1];						// Begin position of the constraint (always 0)
	std::unique_ptr<int[]> matind;		// Position of each element in constraint matrix
	std::unique_ptr<double[]> matval;	// Value of each element in constraint matrix
	char type[1];						// Type of variable (integer 'I', binary 'B', fractional 'C')
	int nonzeros = 0;					// To calculate number of nonzero coefficients in each constraint


	// Create arrays
	matind = std::make_unique<int[]>(2000);
	matval = std::make_unique<double[]>(2000);

	// Create the problem
	pricing_problem = CPXcreateprob(env, &status, "pricing_problem");
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function build_pricing_problem(). \nCouldn't create the CPLEX problem. \nReason: " + std::string(error_text));
	}

	// Problem is minimization
	CPXchgobjsen(env, pricing_problem, CPX_MIN);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function build_pricing_problem(). \nCouldn't set the problem type to minimization. \nReason: " + std::string(error_text));
	}

	// VARIABLES
	// Add the a_lr variables
	for (int l = 0; l < nb_lectures; ++l)
	{
		for (int r = 0; r < nb_rooms; ++r)
		{
			std::string name = "a_" + std::to_string(l + 1) + "_" + std::to_string(r + 1);
			colname[0] = const_cast<char*>(name.c_str());

			obj[0] = 0; // objective function coefficient of zero (this will be changed to the value of (c_lt - mu_l) when solving the problem

			lb[0] = 0;
			ub[0] = 1;
			type[0] = 'B'; // binary variable

			status = CPXnewcols(env, pricing_problem, 1, obj, lb, ub, type, colname); // Generate columns (the variables) 
			if (status != 0)
			{
				CPXgeterrorstring(env, status, error_text);
				throw std::runtime_error("Error in function build_pricing_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
			}
		}
	}

	// CONSTRAINTS
	// Constraint set 1: at most one lecture per room 
	for (int r = 0; r < nb_rooms; ++r) // forall(r in R)
	{
		sense[0] = 'L';	// constraint is '<='
		rhs[0] = 1;		// right-hand side of constraint equals 1

		std::string name = "At_most_one_lecture_in_room_" + std::to_string(r + 1);
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;	// always 0
		nonzeros = 0;	// number of coefficients in constraint different from 0

		for (int l = 0; l < nb_lectures; ++l) // sum(l in L)
		{
			matind[nonzeros] = l * nb_rooms + r;	// variable a_lr is at position l*nb_rooms + r in the constraint matrix
			matval[nonzeros] = 1;					// a_lr has a coefficient of 1 in this constraint
			++nonzeros;		// increase variable nonzeros by 1, as we have a variable with nonzero coefficient in the constraint
		}

		// add the constraint to CPLEX
		status = CPXaddrows(env, pricing_problem, 0, 1, nonzeros, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function build_pricing_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}
	}

	// Constraint set 2: exactly one column selected for every timeslot
	for (int p = 0; p < nb_professors; ++p) // forall(p in P)
	{
		sense[0] = 'L';	// constraint is '<='
		rhs[0] = 1;		// right-hand side of constraint equals 1

		std::string name = "Professor_" + std::to_string(p + 1) + "_teaches_at_most_one_lecture";
		rowname[0] = const_cast<char*>(name.c_str());

		matbeg[0] = 0;	// always 0
		nonzeros = 0;	// number of coefficients in constraint different from 0

		for (int l = 0; l < nb_lectures; ++l) // sum(l in L)
		{
			if (professors_lecture.at(p * nb_lectures + l) == 1) // only lectures that are taught by this professor
			{
				for (int r = 0; r < nb_rooms; ++r) // sum(r in R)
				{
					matind[nonzeros] = l * nb_rooms + r;	// variable a_lr is at position l*nb_rooms + r in the constraint matrix
					matval[nonzeros] = 1;					// a_lr has a coefficient of 1 in this constraint
					++nonzeros;		// increase variable nonzeros by 1, as we have a variable with nonzero coefficient in the constraint
				}
			}
		}

		// add the constraint to CPLEX
		status = CPXaddrows(env, pricing_problem, 0, 1, nonzeros, rhs, sense, matbeg, matind.get(), matval.get(), NULL, rowname);
		if (status != 0)
		{
			CPXgeterrorstring(env, status, error_text);
			throw std::runtime_error("Error in function build_pricing_problem(). \nCouldn't add constraint " + name + ". \nReason: " + std::string(error_text));
		}
	}


	// Write the problem to an LP-file, so that we can check whether it is correct
	status = CPXwriteprob(env, pricing_problem, "pricing_problem.lp", NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function build_pricing_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
	}
}





/* ==================================================================
						SOLVE THE MASTER PROBLEM
   ================================================================== */

void solve_master_problem()
{
	int status = 0;
	int solstat;

	// Assign memory for solution
	solution_masterproblem = std::make_unique<double[]>(CPXgetnumcols(env, master_problem));


	// Optimize the problem
	std::cout << "\n\nCPLEX is solving the master problem ... ";
	status = CPXlpopt(env, master_problem);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function solve_master_problem(). \nCPXlpopt failed. \nReason: " + std::string(error_text));
	}

	// Get the solution
	status = CPXsolution(env, master_problem, &solstat, &objective_value_masterproblem, solution_masterproblem.get(), dual_prices.get(), NULL, NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function solve_master_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
	}


	// Check the solution status
	bool solution_exists;
	std::string solution_info;

	if (solstat == CPX_STAT_OPTIMAL) {
		solution_exists = true;
		solution_info = "Optimal solution found";
	}
	else if (solstat == CPXMIP_INFEASIBLE) {
		solution_exists = false;
		solution_info = "Problem is infeasible";
	}
	else if (solstat == CPX_STAT_INFEASIBLE) {
		solution_exists = false;
		solution_info = "Problem is infeasible";
	}
	else if (solstat == CPX_STAT_UNBOUNDED) {
		solution_exists = false;
		solution_info = "Problem is unbounded";
	}
	else if (solstat == CPX_STAT_INForUNBD) {
		solution_exists = false;
		solution_info = "Problem is infeasible or unbounded";
	}
	else {
		solution_exists = false;
		solution_info = "Other reason for failure";
	}

	if (!solution_exists)
		throw std::runtime_error("CPLEX failed solving the master problem. Reason: " + solution_info);
	

	std::cout << "\nCPLEX has finished: " << solution_info << "\n";
}





/* ==================================================================
					CHANGE COEFFICIENT PRICING PROBLEM
   ================================================================== */

void change_coefficients_pricing_problem(int timeslot)
{
	int nonzeros = 0;

	// coefficients for a_lr
	for (int l = 0; l < nb_lectures; ++l)
	{
		for (int r = 0; r < nb_rooms; ++r)
		{
			double value = cost_lecture_timeslot.at(l * nb_timeslots + timeslot) - dual_prices[l];
			new_coefficients_pricingproblem[l * nb_rooms + r] = value;
			++nonzeros;
		}
	}

	int status = CPXchgobj(env, pricing_problem, nonzeros, indices_variables_pricingproblem.get(), new_coefficients_pricingproblem.get());
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function change_coefficients_pricing_problem(). \nCouldn't change objective function coefficients.\nReason: " + std::string(error_text));
	}

	// Write the problem to an LP-file, so that we can check whether it is correct
	status = CPXwriteprob(env, pricing_problem, "pricing_problem.lp", NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function change_coefficients_pricing_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
	}
}





/* ==================================================================
						SOLVE THE PRICING PROBLEM
   ================================================================== */

double solve_pricing_problem(int timeslot)
{
	int status = 0;
	int solstat;

	// Assign memory for solution
	std::unique_ptr<double[]> solution_CPLEX = std::make_unique<double[]>(CPXgetnumcols(env, pricing_problem));


	// Optimize the problem
	std::cout << "\n\nCPLEX is solving the pricing problem for timeslot " << timeslot + 1 << " ... ";
	status = CPXmipopt(env, pricing_problem);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function solve_pricing_problem(). \nCPXlpopt failed. \nReason: " + std::string(error_text));
	}

	// Get the solution
	status = CPXsolution(env, pricing_problem, &solstat, &objective_value_pricingproblem, solution_CPLEX.get(), NULL, NULL, NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function solve_pricing_problem(). \nCPXsolution failed. \nReason: " + std::string(error_text));
	}


	// Check the solution status
	bool solution_exists;
	std::string solution_info;

	if (solstat == CPXMIP_OPTIMAL) {
		solution_exists = true;
		solution_info = "Optimal solution found";
	}
	else if (solstat == CPXMIP_OPTIMAL_TOL) {
		solution_exists = true;
		solution_info = "Solution found within tolerance limit";
	}
	else if (solstat == CPXMIP_TIME_LIM_FEAS) {
		solution_exists = true;
		solution_info = "Time limit exceeded";
	}
	else if (solstat == CPXMIP_MEM_LIM_FEAS) {
		solution_exists = true;
		solution_info = "Tree memory limit exceeded";
	}
	else if (solstat == CPXMIP_INFEASIBLE) {
		solution_exists = false;
		solution_info = "Problem is infeasible";
	}
	else if (solstat == CPXMIP_INFEASIBLE) {
		solution_exists = false;
		solution_info = "Problem is infeasible";
	}
	else if (solstat == CPXMIP_UNBOUNDED) {
		solution_exists = false;
		solution_info = "Problem is unbounded";
	}
	else if (solstat == CPXMIP_INForUNBD) {
		solution_exists = false;
		solution_info = "Problem is infeasible or unbounded";
	}
	else if (solstat == CPXMIP_TIME_LIM_INFEAS) {
		solution_exists = false;
		solution_info = "Time limit exceeded";
	}
	else if (solstat == CPXMIP_MEM_LIM_INFEAS) {
		solution_exists = false;
		solution_info = "Tree memory limit exceeded";
	}
	else {
		solution_exists = false;
		solution_info = "Other reason for failure";
	}

	if (!solution_exists)
		throw std::runtime_error("CPLEX failed solving the pricing problem. Reason: " + solution_info);


	std::cout << "\nCPLEX has finished: " << solution_info << "\n";

	// calculate cost of new column and fill values of new column
	cost_new_column = 0;
	for (int l = 0; l < nb_lectures; ++l)
	{
		bool lecture_included = false;
		for (int r = 0; r < nb_rooms; ++r)
		{
			cost_new_column += cost_lecture_timeslot.at(l * nb_timeslots + timeslot) * solution_CPLEX[l*nb_rooms + r];
			if (solution_CPLEX[l*nb_rooms + r] > 0.99)
				lecture_included = true;
		}
		if (lecture_included)
			new_column_a.at(l) = 1;
		else
			new_column_a.at(l) = 0;
	}

	return objective_value_pricingproblem - dual_prices[nb_lectures + timeslot];
}





/* ==================================================================
                      ADD COLUMN TO MASTER PROBLEM
   ================================================================== */

double add_column_to_master_problem(int timeslot)
{
	int status = 0;
	double obj[1];						// Objective function coefficient of variables
	double lb[1];						// Lower bounds of variables
	double ub[1];						// Upper bounds of variables
	char *colname[1];					// Variable names
	int matbeg[1];						// Begin position of the constraint (always 0)
	int matind[2000];					// Position of each element in constraint matrix
	double matval[2000];				// Value of each element in constraint matrix
	int nonzeros = 0;					// To calculate number of nonzero coefficients in each constraint

	
	// Add the new column
	std::string name = "z_" + std::to_string(timeslot + 1) + "_" + std::to_string(nb_columns_added + 1);
	colname[0] = const_cast<char*>(name.c_str());

	matbeg[0] = 0;

	obj[0] = cost_new_column; 

	lb[0] = 0;
	ub[0] = 1;

	// Constraint set 1: every lecture assigned to exactly one timeslot
	for (int l = 0; l < nb_lectures; ++l)
	{
		matind[nonzeros] = l;
		matval[nonzeros] = new_column_a.at(l);
		++nonzeros;
	}

	// Constraint set 2: exactly one column per timeslot selected
	matind[nonzeros] = nb_lectures + timeslot;
	matval[nonzeros] = 1;
	++nonzeros;

	status = CPXaddcols(env, master_problem, 1, nonzeros, obj, matbeg, matind, matval, lb, ub, colname);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function add_column_to_master_problem(). \nCouldn't add variable " + name + ". \nReason: " + std::string(error_text));
	}

	// Write the problem to an LP-file, so that we can check whether it is correct
	status = CPXwriteprob(env, master_problem, "master_problem.lp", NULL);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function add_column_to_master_problem(). \nFailed to write the problem to a file. \nReason: " + std::string(error_text));
	}
}





/* ==================================================================
							CLEAR CPLEX
   ================================================================== */

void clear_cplex()
{
	int status = 0;

	// Free the master problem
	status = CPXfreeprob(env, &master_problem);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function clear_cplex(). \nCouldn't free the CPLEX master problem. \nReason: " + std::string(error_text));
	}

	// Free the pricing problem
	status = CPXfreeprob(env, &pricing_problem);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function clear_cplex(). \nCouldn't free the CPLEX pricing problem. \nReason: " + std::string(error_text));
	}

	// Close the cplex environment
	status = CPXcloseCPLEX(&env);
	if (status != 0)
	{
		CPXgeterrorstring(env, status, error_text);
		throw std::runtime_error("Error in function clear_cplex(). \nCouldn't close CPLEX environment. \nReason: " + std::string(error_text));
	}
}





/* ==================================================================
                            WRITE OUTPUT
   ================================================================== */

void write_output()
{
	// To file
	{
		std::ofstream file;
		file.open("output_column_generation.txt");
		if (!file.is_open())
			throw std::runtime_error("Couldn't open output file!");

		file << "Computation time (seconds)\t" << computation_time;
		file << "\nNumber of columns added\t" << nb_columns_added;
		file << "\nObjective value optimal solution\t" << objective_value_masterproblem;
	}

	// To screen
	{
		std::cout << "\n\n\nColumn generation has finished";
		std::cout << "\nComputation time (seconds): " << computation_time;
		std::cout << "\nNumber of columns added: " << nb_columns_added;
		std::cout << "\nObjective value optimal solution: " << objective_value_masterproblem;
	}
}





/* ==================================================================
						COLUMN GENERATION LOOP
   ================================================================== */

void column_generation()
{
	// start a timer
	auto start_time = std::chrono::system_clock::now();

	initialize_cplex(); 
	build_master_problem();
	build_pricing_problem();

	// column generation loop
	nb_columns_added = 0;
	int iteration = 0;
	while (true)
	{
		++iteration;
		std::cout << "\n\n\n\n\nColumn generation iteration: " << iteration << "\n";
		
		solve_master_problem();

		// for every timeslot, we need to solve a pricing problem
		bool new_columns_found = false; // remember whether we found new columns or not
		for (int t = 0; t < nb_timeslots; ++t)
		{
			change_coefficients_pricing_problem(t);
			double reduced_cost = solve_pricing_problem(t);

			// if the reduced cost of the problem is < 0 (+ rounding errors)
			if (reduced_cost < -0.0001)
			{
				std::cout << "New column found with reduced cost: " << reduced_cost;

				// if we have found a new column with negative reduced cost, we add it to the master problem
				add_column_to_master_problem(t);

				// we have found a new column
				new_columns_found = true;
				++nb_columns_added;
			}
		}

		// if we didn't find any new columns, we can stop
		if (!new_columns_found)
			break; // exit the column generation loop ("while(true)")
	}

	// Get the elapsed time
	std::chrono::duration<double, std::ratio<1,1>> elapsed_time = std::chrono::system_clock::now() - start_time;
	computation_time = elapsed_time.count();

	// write output to screen and file
	write_output();

	// clear cplex
	clear_cplex();


}





/* ==================================================================
                                 MAIN
   ================================================================== */

// Program execution always starts at "main"
int main()
{
	try
	{
		// Ask the user for the name of the input file
		std::string filename;
		std::cout << "Give the filename for the problem instance: ";
		std::cin >> filename;

		// First read the problem data from the specified file
		read_problem_data_from_file(filename);

		// Then try to solve the problem with column generation
		column_generation();
	}
	catch (const std::exception& ex)
	{
		// An error occurred: write to screen what went wrong
		std::cout << "\n\n" << ex.what();
	}

	std::cout << "\n\n\n\n\nPress enter to exit the program ... ";
	getchar();
	getchar();
	return 0; // program ends here
}



