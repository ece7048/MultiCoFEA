
#include <OpenSim/OpenSim.h>
#include <OpenSim/Analyses/BodyKinematics.h>
#include <OpenSim/Tools/AnalyzeTool.h>
#include <OpenSim/Simulation/SimbodyEngine/Body.h>
#include <stdlib.h>
#include <ctime>
#include <exception>
#include "Settings.h"
#include "INIReader.h"
#include <iostream>
#include <fstream>
#include <algorithm>
#include <math.h>
#include <direct.h>
#include "Visualizerfebgeo.h"
#include "TSC.h"

Visualizerfebgeo::Visualizerfebgeo(void)
{
}

Visualizerfebgeo::~Visualizerfebgeo(void)
{
}

void Visualizerfebgeo::XMLwrite(){
	ofstream opengeo;
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
	string geof = ini.Get("FEBIOSTEP", "GEOF", "");
	string geos = ini.Get("FEBIOSTEP", "GEOS", "");
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
	//char itter = itteration + '0';
	////create a new folder for the analysis/////////
	
	
	opengeo = ofstream(resultDir1 + "/geopensim.osim", ofstream::out);

	opengeo << "<?xml version=" <<'"'<<"1.0"<<'"'<<" encoding=" <<'"'<<"UTF-8" << '"' << "?>" << endl;
	opengeo << "<OpenSimDocument Version ="<<'"'<<"30000"<<'"'<<">" << endl;
	opengeo <<"	<Model name="<<'"'<<"femgeo"<<'"'<<">"<< endl;
		opengeo << "		<!--See the credits section below for information about this model's authors, data sources, intended uses, " << endl; 
		opengeo << "		and more.See the publications section for the paper(s) you should cite when using this model.Do not remove either section if you modify or add to this model." << endl; 
		opengeo << "		If you are this model's author(s), add or update the credits and publications sections before distributing your model.-->" << endl;
		opengeo << "		<credits>Mamalakis.M.</credits>" << endl;
		opengeo << "		<publications>Notes:</publications>" << endl;
		opengeo << "		<length_units>meters</length_units>" << endl;
		opengeo << "		<force_units>N</force_units>" << endl;
		opengeo << "		<!--Acceleration due to gravity.-->" << endl;
		opengeo << "		<gravity> 0 -9.80665 0</gravity>" << endl;
		opengeo << "		<!--Bodies in the model.-->" << endl;



		/// write ground body////
		opengeo << "		<BodySet>" << endl;
		opengeo << "			<objects>" << endl;

		opengeo << "				<Body name =" << '"' << "ground" << '"' << ">" << endl;
		opengeo << "					<mass>0</mass>" << endl;
		opengeo << "					<mass_center> 0 0 0</mass_center>" << endl;
		opengeo << "					<inertia_xx>0</inertia_xx>" << endl;
		opengeo << "					<inertia_yy>0</inertia_yy>" << endl;
		opengeo << "					<inertia_zz>0</inertia_zz>" << endl;
		opengeo << "					<inertia_xy>0</inertia_xy>" << endl;
		opengeo << "					<inertia_xz>0</inertia_xz>" << endl;
		opengeo << "					<inertia_yz>0</inertia_yz>" << endl;
		opengeo << "					<!--Joint that connects this body with the parent body.-->" << endl;
		opengeo << "					<Joint/>" << endl;
		opengeo << "					<VisibleObject>" << endl;
		opengeo << "						<!--Set of geometry files and associated attributes, allow.vtp, .stl, .obj-->" << endl;
		opengeo << "						<GeometrySet>" << endl;
		opengeo << "							<objects>" << endl;
		opengeo << "								<DisplayGeometry>" << endl;
		opengeo << "									<!--Name of geometry file.vtp, .stl, .obj-->" << endl;
		opengeo << "									<geometry_file></geometry_file>" << endl;
		opengeo << "									<!--Color used to display the geometry when visible-->" << endl;
		opengeo << "									<color> 1 1 1</color>" << endl;
		opengeo << "									<!--Name of texture file.jpg, .bmp-->" << endl;
		opengeo << "									<texture_file/>" << endl;
		opengeo << "									<!--in body transform specified as 3 rotations(rad) followed by 3 translations rX rY rZ tx ty tz-->" << endl;
		opengeo << "									<transform> -0 0 -0 0 0 0</transform>" << endl;
		opengeo << "									<!--Three scale factors for display purposes : scaleX scaleY scaleZ-->" << endl;
		opengeo << "									<scale_factors> 1 1 1</scale_factors>" << endl;
		opengeo << "									<!--Display Pref. 0:Hide 1 : Wire 3 : Flat 4 : Shaded-->" << endl;
		opengeo << "									<display_preference>4</display_preference>" << endl;
		opengeo << "									<!--Display opacity between 0.0 and 1.0-->" << endl;
		opengeo << "									<opacity>1</opacity>" << endl;
		opengeo << "								</DisplayGeometry>" << endl;
		opengeo << "							</objects>" << endl;
		opengeo << "							<groups/>" << endl;
		opengeo << "						</GeometrySet>" << endl;
		opengeo << "						<!--Three scale factors for display purposes : scaleX scaleY scaleZ-->" << endl;
		opengeo << "						<scale_factors> 1 1 1</scale_factors>" << endl;
		opengeo << "						<!--transform relative to owner specified as 3 rotations(rad) followed by 3 translations rX rY rZ tx ty tz-->" << endl;
		opengeo << "						<transform> -0 0 -0 0 0 0</transform>" << endl;
		opengeo << "						<!--Whether to show a coordinate frame-->" << endl;
		opengeo << "						<show_axes>false</show_axes>" << endl;
		opengeo << "						<!--Display Pref. 0:Hide 1 : Wire 3 : Flat 4 : Shaded Can be overriden for individual geometries-->" << endl;
		opengeo << "						<display_preference>4</display_preference>" << endl;
		opengeo << "					</VisibleObject>" << endl;
		opengeo << "					<WrapObjectSet>" << endl;
		opengeo << "						<objects/>" << endl;
		opengeo << "						<groups/>" << endl;
		opengeo << "					</WrapObjectSet>" << endl;
		opengeo << "				</Body>" << endl;
	

		//write first rigid body ans a 6dof in ground//

		opengeo << "				<Body name="<<'"'<<bd1<<'"'<<">"<< endl;
		opengeo << "					<mass>0</mass>" << endl;
		opengeo << "					<mass_center> 0 0 0</mass_center>" << endl;
		opengeo << "					<inertia_xx>0</inertia_xx>" << endl;
		opengeo << "					<inertia_yy>0</inertia_yy>" << endl;
		opengeo << "					<inertia_zz>0</inertia_zz>" << endl;
		opengeo << "					<inertia_xy>0</inertia_xy>" << endl;
		opengeo << "					<inertia_xz>0</inertia_xz>" << endl;
		opengeo << "					<inertia_yz>0</inertia_yz>" << endl;
		opengeo << "					<!--Joint that connects this body with the parent body.-->" << endl;
		opengeo << "					<Joint>" << endl;
	
		opengeo << "			<CustomJoint name="<<'"'<<"ground_"<<bd1<<'"'<<">" << endl;
		opengeo << "	<!--Name of the parent body to which this joint connects its owner body.-->" << endl;
		opengeo << "	<parent_body>ground</parent_body>" << endl;
		opengeo << "	<!--Location of the joint in the parent body specified in the parent reference frame.Default is(0, 0, 0).-->" << endl;
		opengeo << "	<location_in_parent>0 0 0</location_in_parent>" << endl;
		opengeo << "	<!--Orientation of the joint in the parent body specified in the parent reference frame.Euler XYZ body - fixed rotation angles are used to express the orientation.Default is(0, 0, 0).-->" << endl;
		opengeo << "	<orientation_in_parent>0 0 0</orientation_in_parent>" << endl;
		opengeo << "	<!--Location of the joint in the child body specified in the child reference frame.For SIMM models, this vector is always the zero vector(i.e., the body reference frame coincides with the joint). -->" << endl;
		opengeo << "	<location>0 0 0</location>" << endl;
		opengeo << "	<!--Orientation of the joint in the owing body specified in the owning body reference frame.Euler XYZ body - fixed rotation angles are used to express the orientation. -->" << endl;
		opengeo << "	<orientation>0 0 0</orientation>" << endl;
		opengeo << "	<!--Set holding the generalized coordinates(q's) that parmeterize this joint.-->" << endl;
		opengeo << "	<CoordinateSet>" << endl;
		opengeo << "	<objects>" << endl;
			opengeo << "	<Coordinate name=" << '"' << bd1 << "_tilt" << '"' << ">" << endl;
			opengeo << "	<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
			opengeo << "	<motion_type>rotational</motion_type>" << endl;
			opengeo << "	<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
			opengeo << "	<default_value>0</default_value>" << endl;
			opengeo << "	<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
			opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-2 2</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
			opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
			opengeo << "<Coordinate name=" << '"' << bd1 << "_list" << '"' << ">" << endl;
			opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
			opengeo << "<motion_type>rotational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-2 2</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
			opengeo << "</Coordinate>" << endl;
			opengeo << "<Coordinate name=" <<'"'<< bd1 << "_rotation" << '"' << ">" << endl;
			opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
			opengeo << "<motion_type>rotational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-2 2</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
			opengeo << "<Coordinate name=" <<'"'<< bd1 << "_tx" << '"' << ">" << endl;
			opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
			opengeo << "<motion_type>translational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-0.01 0.01</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
			opengeo << "<clamped>false</clamped>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
			opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
			opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
			opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
			opengeo << "<Coordinate name=" <<'"'<< bd1 << "_ty" << '"' << ">" << endl;
			opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>translational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
			opengeo << "<default_value>0</default_value>" << endl;
			opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
			opengeo << "<default_speed_value>0</default_speed_value>" << endl;
			opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
			opengeo << "<range>-0.01 0.01</range>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
			opengeo << "<clamped>false</clamped>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
			opengeo << "<locked>false</locked>" << endl;
			opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
			opengeo << "<prescribed_function/>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
			opengeo << "<prescribed>false</prescribed>" << endl;
			opengeo << "</Coordinate>" << endl;
			opengeo << "<Coordinate name=" <<'"'<< bd1 << "_tz" << '"' << ">" << endl;
			opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>translational</motion_type>" << endl;
			opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
			opengeo << "<default_value>0</default_value>" << endl;
			opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
			opengeo << "<default_speed_value>0</default_speed_value>" << endl;
			opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
			opengeo << "<range>-0.01 0.01</range>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
			opengeo << "<clamped>false</clamped>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
			opengeo << "<locked>false</locked>" << endl;
			opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
			opengeo << "<prescribed_function/>" << endl;
			opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
			opengeo << "<prescribed>false</prescribed>" << endl;
			opengeo << "</Coordinate>" << endl;
			opengeo << "</objects>" << endl;
			opengeo << "<groups/>" << endl;
			opengeo << "</CoordinateSet>" << endl;
			opengeo << "<!--Whether the joint transform defines parent->child or child->parent.-->" << endl;
			opengeo << "<reverse>false</reverse>" << endl;
			opengeo << "<!--Defines how the child body moves with respect to the parent as a function of the generalized coordinates.-->" << endl;
			opengeo << "<SpatialTransform>" << endl;
			opengeo << "<!--3 Axes for rotations are listed first.-->" << endl;
			opengeo << "<TransformAxis name="<< '"' << "rotation1" << '"' << ">" << endl;
			opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
			opengeo << "<coordinates>" << bd1 << "_tilt</coordinates>" << endl;
			opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
			opengeo << "<axis>0 0 1</axis>" << endl;
			opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
			opengeo << "<function>" << endl;
			opengeo << "<LinearFunction>" << endl;
			opengeo << "<coefficients> 1 0</coefficients>" << endl;
			opengeo << "</LinearFunction>" << endl;
			opengeo << "</function>" << endl;
			opengeo << "</TransformAxis>" << endl;
			opengeo << "<TransformAxis name="<<'"'<<"rotation2"<<'"'<<">" << endl;
			opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
			opengeo << "<coordinates>" << bd1 << "_list</coordinates>" << endl;
			opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
			opengeo << "<axis>1 0 0</axis>" << endl;
			opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
			opengeo << "<function>" << endl;
			opengeo << "<LinearFunction>" << endl;
			opengeo << "<coefficients> 1 0</coefficients>" << endl;
			opengeo << "</LinearFunction>" << endl;
			opengeo << "</function>" << endl;
			opengeo << "</TransformAxis>" << endl;
			opengeo << "<TransformAxis name="<<'"'<<"rotation3"<<'"'<<">" << endl;
			opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
			opengeo << "<coordinates>"<<bd1<<"_rotation</coordinates>" << endl;
			opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
			opengeo << "<axis>0 1 0</axis>" << endl;
			opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
			opengeo << "<function>" << endl;
			opengeo << "<LinearFunction>" << endl;
			opengeo << "<coefficients> 1 0</coefficients>" << endl;
			opengeo << "</LinearFunction>" << endl;
			opengeo << "</function>" << endl;
			opengeo << "</TransformAxis>" << endl;
			opengeo << "<!--3 Axes for translations are listed next.-->" << endl;
			opengeo << "<TransformAxis name="<<'"'<<"translation1"<<'"'<<">" << endl;
			opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
			opengeo << "<coordinates>" << bd1 << "_tx</coordinates>" << endl;
			opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
			opengeo << "<axis>1 0 0</axis>" << endl;
			opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
			opengeo << "<function>" << endl;
			opengeo << "<LinearFunction>" << endl;
			opengeo << "<coefficients> 1 0</coefficients>" << endl;
			opengeo << "</LinearFunction>" << endl;
			opengeo << "</function>" << endl;
			opengeo << "</TransformAxis>" << endl;
			opengeo << "<TransformAxis name="<<'"'<< "translation2"<<'"'<<">" << endl;
			opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
			opengeo << "<coordinates>" << bd1 << "_ty</coordinates>" << endl;
			opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
			opengeo << "<axis>0 1 0</axis>" << endl;
			opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
			opengeo << "<function>" << endl;
			opengeo << "<LinearFunction>" << endl;
			opengeo << "<coefficients> 1 0</coefficients>" << endl;
			opengeo << "</LinearFunction>" << endl;
			opengeo << "</function>" << endl;
			opengeo << "</TransformAxis>" << endl;
			opengeo << "<TransformAxis name = "<<'"'<<"translation3"<<'"'<<">" << endl;
			opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
			opengeo << "<coordinates>"<<bd1<<"_tz</coordinates>" << endl;
			opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
			opengeo << "<axis>0 0 1</axis>" << endl;
			opengeo << "<!--Transform function of the generalized coordinates used to represent the amount of transformation along a specified axis.-->" << endl;
			opengeo << "<function>" << endl;
			opengeo << "<LinearFunction>" << endl;
			opengeo << "<coefficients> 1 0</coefficients>" << endl;
			opengeo << "</LinearFunction>" << endl;
			opengeo << "</function>" << endl;
			opengeo << "</TransformAxis>" << endl;
			opengeo << "</SpatialTransform>" << endl;
			opengeo << "</CustomJoint>" << endl;
			opengeo << "</Joint>" << endl;
		opengeo << "					<VisibleObject>" << endl;
		opengeo << "						<!--Set of geometry files and associated attributes, allow.vtp, .stl, .obj-->" << endl;
		opengeo << "						<GeometrySet>" << endl;
		opengeo << "							<objects>" << endl;
		opengeo << "								<DisplayGeometry>" << endl;
		opengeo << "									<!--Name of geometry file.vtp, .stl, .obj-->" << endl;
		opengeo << "									<geometry_file>"<<geof<<"</geometry_file>" << endl;
		opengeo << "									<!--Color used to display the geometry when visible-->" << endl;
				opengeo << "									<color> 1 1 1</color>" << endl;
				opengeo << "									<!--Name of texture file.jpg, .bmp-->" << endl;
				opengeo << "									<texture_file/>" << endl;
				opengeo << "									<!--in body transform specified as 3 rotations(rad) followed by 3 translations rX rY rZ tx ty tz-->" << endl;
				opengeo << "									<transform> -0 0 -0 0 0 0</transform>" << endl;
				opengeo << "									<!--Three scale factors for display purposes : scaleX scaleY scaleZ-->" << endl;
				opengeo << "									<scale_factors> 1 1 1</scale_factors>" << endl;
				opengeo << "									<!--Display Pref. 0:Hide 1 : Wire 3 : Flat 4 : Shaded-->" << endl;
				opengeo << "									<display_preference>4</display_preference>" << endl;
				opengeo << "									<!--Display opacity between 0.0 and 1.0-->" << endl;
				opengeo << "									<opacity>1</opacity>" << endl;
				opengeo << "								</DisplayGeometry>" << endl;
				opengeo << "							</objects>" << endl;
				opengeo << "							<groups/>" << endl;
				opengeo << "						</GeometrySet>" << endl;
				opengeo << "						<!--Three scale factors for display purposes : scaleX scaleY scaleZ-->" << endl;
				opengeo << "						<scale_factors> 1 1 1</scale_factors>" << endl;
				opengeo << "						<!--transform relative to owner specified as 3 rotations(rad) followed by 3 translations rX rY rZ tx ty tz-->" << endl;
				opengeo << "						<transform> -0 0 -0 0 0 0</transform>" << endl;
				opengeo << "						<!--Whether to show a coordinate frame-->" << endl;
				opengeo << "						<show_axes>false</show_axes>" << endl;
				opengeo << "						<!--Display Pref. 0:Hide 1 : Wire 3 : Flat 4 : Shaded Can be overriden for individual geometries-->" << endl;
				opengeo << "						<display_preference>4</display_preference>" << endl;
				opengeo << "					</VisibleObject>" << endl;
				opengeo << "					<WrapObjectSet>" << endl;
				opengeo << "						<objects/>" << endl;
				opengeo << "						<groups/>" << endl;
				opengeo << "					</WrapObjectSet>" << endl;
				opengeo << "				</Body>" << endl;
			
			
				opengeo << "				<Body name ="<<'"'<<bd2<< '"' << ">" << endl;
				opengeo << "					<mass>0</mass>" << endl;
				opengeo << "					<mass_center> 0 0 0</mass_center>" << endl;
				opengeo << "					<inertia_xx>0</inertia_xx>" << endl;
				opengeo << "					<inertia_yy>0</inertia_yy>" << endl;
				opengeo << "					<inertia_zz>0</inertia_zz>" << endl;
				opengeo << "					<inertia_xy>0</inertia_xy>" << endl;
				opengeo << "					<inertia_xz>0</inertia_xz>" << endl;
				opengeo << "					<inertia_yz>0</inertia_yz>" << endl;
				opengeo << "					<!--Joint that connects this body with the parent body.-->" << endl;
				opengeo << "					<Joint>" << endl;

				opengeo << "			<CustomJoint name=" << '"' << "ground_" << bd2 << '"' << ">" << endl;
				opengeo << "	<!--Name of the parent body to which this joint connects its owner body.-->" << endl;
				opengeo << "	<parent_body>ground</parent_body>" << endl;
				opengeo << "	<!--Location of the joint in the parent body specified in the parent reference frame.Default is(0, 0, 0).-->" << endl;
				opengeo << "	<location_in_parent>0 0 0</location_in_parent>" << endl;
				opengeo << "	<!--Orientation of the joint in the parent body specified in the parent reference frame.Euler XYZ body - fixed rotation angles are used to express the orientation.Default is(0, 0, 0).-->" << endl;
				opengeo << "	<orientation_in_parent>0 0 0</orientation_in_parent>" << endl;
				opengeo << "	<!--Location of the joint in the child body specified in the child reference frame.For SIMM models, this vector is always the zero vector(i.e., the body reference frame coincides with the joint). -->" << endl;
				opengeo << "	<location>0 0 0</location>" << endl;
				opengeo << "	<!--Orientation of the joint in the owing body specified in the owning body reference frame.Euler XYZ body - fixed rotation angles are used to express the orientation. -->" << endl;
				opengeo << "	<orientation>0 0 0</orientation>" << endl;
				opengeo << "	<!--Set holding the generalized coordinates(q's) that parmeterize this joint.-->" << endl;
				opengeo << "	<CoordinateSet>" << endl;
				opengeo << "	<objects>" << endl;
				opengeo << "	<Coordinate name=" << '"' << bd2 << "_tilt" << '"' << ">" << endl;
				opengeo << "	<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "	<motion_type>rotational</motion_type>" << endl;
				opengeo << "	<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "	<default_value>0</default_value>" << endl;
				opengeo << "	<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-2 2</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
				opengeo << "<Coordinate name=" << '"' << bd2 << "_list" << '"' << ">" << endl;
				opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>rotational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-2 2</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
				opengeo << "<Coordinate name=" << '"' << bd2 << "_rotation" << '"' << ">" << endl;
				opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>rotational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-2 2</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
				opengeo << "<Coordinate name=" << '"' << bd2 << "_tx" << '"' << ">" << endl;
				opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>translational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-0.01 0.01</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
				opengeo << "<Coordinate name=" << '"' << bd2 << "_ty" << '"' << ">" << endl;
				opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>translational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-0.01 0.01</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
				opengeo << "<Coordinate name=" << '"' << bd2 << "_tz" << '"' << ">" << endl;
				opengeo << "<!--Coordinate can describe rotational, translational, or coupled motion.Defaults to rotational.-->" << endl;
				opengeo << "<motion_type>translational</motion_type>" << endl;
				opengeo << "<!--The value of this coordinate before any value has been set.Rotational coordinate value is in radians and Translational in meters.-->" << endl;
				opengeo << "<default_value>0</default_value>" << endl;
				opengeo << "<!--The speed value of this coordinate before any value has been set.Rotational coordinate value is in rad / s and Translational in m / s.-->" << endl;
				opengeo << "<default_speed_value>0</default_speed_value>" << endl;
				opengeo << "<!--The minimum and maximum values that the coordinate can range between.Rotational coordinate range in radians and Translational in meters.-->" << endl;
				opengeo << "<range>-0.01 0.01</range>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be limited to the range, above.-->" << endl;
				opengeo << "<clamped>false</clamped>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be constrained to the current(e.g. default) value, above.-->" << endl;
				opengeo << "<locked>false</locked>" << endl;
				opengeo << "<!--If specified, the coordinate can be prescribed by a function of time.It can be any OpenSim Function with valid second order derivatives.-->" << endl;
				opengeo << "<prescribed_function/>" << endl;
				opengeo << "<!--Flag indicating whether or not the values of the coordinates should be prescribed according to the function above.It is ignored if the no prescribed function is specified.-->" << endl;
				opengeo << "<prescribed>false</prescribed>" << endl;
				opengeo << "</Coordinate>" << endl;
				opengeo << "</objects>" << endl;
				opengeo << "<groups/>" << endl;
				opengeo << "</CoordinateSet>" << endl;
				opengeo << "<!--Whether the joint transform defines parent->child or child->parent.-->" << endl;
				opengeo << "<reverse>false</reverse>" << endl;
				opengeo << "<!--Defines how the child body moves with respect to the parent as a function of the generalized coordinates.-->" << endl;
				opengeo << "<SpatialTransform>" << endl;
				opengeo << "<!--3 Axes for rotations are listed first.-->" << endl;
				opengeo << "<TransformAxis name=" << '"' << "rotation1" << '"' << ">" << endl;
				opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
				opengeo << "<coordinates>" << bd2 << "_tilt</coordinates>" << endl;
				opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
				opengeo << "<axis>0 0 1</axis>" << endl;
				opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
				opengeo << "<function>" << endl;
				opengeo << "<LinearFunction>" << endl;
				opengeo << "<coefficients> 1 0</coefficients>" << endl;
				opengeo << "</LinearFunction>" << endl;
				opengeo << "</function>" << endl;
				opengeo << "</TransformAxis>" << endl;
				opengeo << "<TransformAxis name=" << '"' << "rotation2" << '"' << ">" << endl;
				opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
				opengeo << "<coordinates>" << bd2 << "_list</coordinates>" << endl;
				opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
				opengeo << "<axis>1 0 0</axis>" << endl;
				opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
				opengeo << "<function>" << endl;
				opengeo << "<LinearFunction>" << endl;
				opengeo << "<coefficients> 1 0</coefficients>" << endl;
				opengeo << "</LinearFunction>" << endl;
				opengeo << "</function>" << endl;
				opengeo << "</TransformAxis>" << endl;
				opengeo << "<TransformAxis name=" << '"' << "rotation3" << '"' << ">" << endl;
				opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
				opengeo << "<coordinates>" << bd2 << "_rotation</coordinates>" << endl;
				opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
				opengeo << "<axis>0 1 0</axis>" << endl;
				opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
				opengeo << "<function>" << endl;
				opengeo << "<LinearFunction>" << endl;
				opengeo << "<coefficients> 1 0</coefficients>" << endl;
				opengeo << "</LinearFunction>" << endl;
				opengeo << "</function>" << endl;
				opengeo << "</TransformAxis>" << endl;
				opengeo << "<!--3 Axes for translations are listed next.-->" << endl;
				opengeo << "<TransformAxis name=" << '"' << "translation1" << '"' << ">" << endl;
				opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
				opengeo << "<coordinates>" << bd2 << "_tx</coordinates>" << endl;
				opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
				opengeo << "<axis>1 0 0</axis>" << endl;
				opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
				opengeo << "<function>" << endl;
				opengeo << "<LinearFunction>" << endl;
				opengeo << "<coefficients> 1 0</coefficients>" << endl;
				opengeo << "</LinearFunction>" << endl;
				opengeo << "</function>" << endl;
				opengeo << "</TransformAxis>" << endl;
				opengeo << "<TransformAxis name=" << '"' << "translation2" << '"' << ">" << endl;
				opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
				opengeo << "<coordinates>" << bd2 << "_ty</coordinates>" << endl;
				opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
				opengeo << "<axis>0 1 0</axis>" << endl;
				opengeo << "<!--Transform function of the generalized coordinates used to        represent the amount of transformation along a specified axis.-->" << endl;
				opengeo << "<function>" << endl;
				opengeo << "<LinearFunction>" << endl;
				opengeo << "<coefficients> 1 0</coefficients>" << endl;
				opengeo << "</LinearFunction>" << endl;
				opengeo << "</function>" << endl;
				opengeo << "</TransformAxis>" << endl;
				opengeo << "<TransformAxis name = " << '"' << "translation3" << '"' << ">" << endl;
				opengeo << "<!--Names of the coordinates that serve as the independent variables         of the transform function.-->" << endl;
				opengeo << "<coordinates>" << bd2 << "_tz</coordinates>" << endl;
				opengeo << "<!--Rotation or translation axis for the transform.-->" << endl;
				opengeo << "<axis>0 0 1</axis>" << endl;
				opengeo << "<!--Transform function of the generalized coordinates used to represent the amount of transformation along a specified axis.-->" << endl;
				opengeo << "<function>" << endl;
				opengeo << "<LinearFunction>" << endl;
				opengeo << "<coefficients> 1 0</coefficients>" << endl;
				opengeo << "</LinearFunction>" << endl;
				opengeo << "</function>" << endl;
				opengeo << "</TransformAxis>" << endl;
				opengeo << "</SpatialTransform>" << endl;
				opengeo << "</CustomJoint>" << endl;
				opengeo << "</Joint>" << endl;
				opengeo << "					<VisibleObject>" << endl;
				opengeo << "						<!--Set of geometry files and associated attributes, allow.vtp, .stl, .obj-->" << endl;
				opengeo << "						<GeometrySet>" << endl;
				opengeo << "							<objects>" << endl;
				opengeo << "								<DisplayGeometry>" << endl;
				opengeo << "									<!--Name of geometry file.vtp, .stl, .obj-->" << endl;
				opengeo << "									<geometry_file>" << geos << "</geometry_file>" << endl;
				opengeo << "									<!--Color used to display the geometry when visible-->" << endl;
				opengeo << "									<color> 1 1 1</color>" << endl;
				opengeo << "									<!--Name of texture file.jpg, .bmp-->" << endl;
				opengeo << "									<texture_file/>" << endl;
				opengeo << "									<!--in body transform specified as 3 rotations(rad) followed by 3 translations rX rY rZ tx ty tz-->" << endl;
				opengeo << "									<transform> -0 0 -0 0 0 0</transform>" << endl;
				opengeo << "									<!--Three scale factors for display purposes : scaleX scaleY scaleZ-->" << endl;
				opengeo << "									<scale_factors> 1 1 1</scale_factors>" << endl;
				opengeo << "									<!--Display Pref. 0:Hide 1 : Wire 3 : Flat 4 : Shaded-->" << endl;
				opengeo << "									<display_preference>4</display_preference>" << endl;
				opengeo << "									<!--Display opacity between 0.0 and 1.0-->" << endl;
				opengeo << "									<opacity>1</opacity>" << endl;
				opengeo << "								</DisplayGeometry>" << endl;
				opengeo << "							</objects>" << endl;
				opengeo << "							<groups/>" << endl;
				opengeo << "						</GeometrySet>" << endl;
				opengeo << "						<!--Three scale factors for display purposes : scaleX scaleY scaleZ-->" << endl;
				opengeo << "						<scale_factors> 1 1 1</scale_factors>" << endl;
				opengeo << "						<!--transform relative to owner specified as 3 rotations(rad) followed by 3 translations rX rY rZ tx ty tz-->" << endl;
				opengeo << "						<transform> -0 0 -0 0 0 0</transform>" << endl;
				opengeo << "						<!--Whether to show a coordinate frame-->" << endl;
				opengeo << "						<show_axes>false</show_axes>" << endl;
				opengeo << "						<!--Display Pref. 0:Hide 1 : Wire 3 : Flat 4 : Shaded Can be overriden for individual geometries-->" << endl;
				opengeo << "						<display_preference>4</display_preference>" << endl;
				opengeo << "					</VisibleObject>" << endl;
				opengeo << "					<WrapObjectSet>" << endl;
				opengeo << "						<objects/>" << endl;
				opengeo << "						<groups/>" << endl;
				opengeo << "					</WrapObjectSet>" << endl;
				opengeo << "				</Body>" << endl;
				opengeo << "			</objects>" << endl;
					opengeo << "			<groups/>" << endl;
					opengeo << "		</BodySet>" << endl;

					opengeo << "		<ConstraintSet>" << endl;
					opengeo << "			<objects/>" << endl;
					opengeo << "			<groups/>" << endl;
					opengeo << "		</ConstraintSet>" << endl;
					opengeo << "		<ForceSet>" << endl;
					opengeo << "			<objects/>" << endl;
					opengeo << "			<groups/>" << endl;
					opengeo << "		</ForceSet>" << endl;
					opengeo << "		<MarkerSet>" << endl;
					opengeo << "			<objects/>" << endl;
					opengeo << "			<groups/>" << endl;
					opengeo << "		</MarkerSet>" << endl;
					opengeo << "		<ContactGeometrySet>" << endl;
					opengeo << "			<objects/>" << endl;
					opengeo << "			<groups/>" << endl;
					opengeo << "		</ContactGeometrySet>" << endl;
					opengeo << "		<ControllerSet name = " << '"' << "Controllers" << '"' << ">" << endl;
							opengeo << "			<objects/>" << endl;
						opengeo << "			<groups/>" << endl;
						opengeo << "		</ControllerSet>" << endl;
						opengeo << "		<ComponentSet name = " << '"' << "MiscComponents" << '"' << ">" << endl;
							opengeo << "			<objects/>" << endl;
						opengeo << "			<groups/>" << endl;
						opengeo << "		</ComponentSet>" << endl;
						opengeo << "		<ProbeSet>" << endl;
						opengeo << "			<objects/>" << endl;
						opengeo << "			<groups/>" << endl;
						opengeo << "		</ProbeSet>" << endl;
						opengeo << "	</Model>" << endl;
							opengeo <<" </OpenSimDocument>" << endl;



	//end of XML writting					

}





//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
//	%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////P.S.  SECTION/////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/*
void Visualizerfebgeo::Run()
{
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
	string modelPath = resultDir1 + "/geopensim.osim";
	string programname = ini.Get("PATH", "PROGRAMMNAME", "");
	string name = ini.Get("PATH", "NAME", "");
	string arg1 = " --console suppress ";

	int count = 0;
		string arg2 = modelPath;
		cout << "" << endl;
		cout << "START THE OPENSIM GUI " << endl;

		string workingFileName = arg2;
	
		string arguments = arg1;

		HANDLE process = ShellExecuteHandler(programname, arguments, name);
		cout << "" << endl;
		cout << "PASS THE .osim FILE AND THE .mot FILE IN OPENSIM GUI " << endl;
		string name1 = "";
		string arguments1 = "";
		HANDLE process1 = Shellfile(resultDir1);

		
		}//void
	


HANDLE Visualizerfebgeo::ShellExecuteHandler(string program, string args, string name)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "open";
	shellInfo.lpFile = name.c_str();
	shellInfo.lpParameters = args.c_str();
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}

HANDLE Visualizerfebgeo::Shellfile(string program)
{
	HANDLE hProcess = NULL;
	SHELLEXECUTEINFO shellInfo;
	::ZeroMemory(&shellInfo, sizeof(shellInfo));
	shellInfo.cbSize = sizeof(shellInfo);
	shellInfo.fMask = SEE_MASK_FLAG_NO_UI | SEE_MASK_NOCLOSEPROCESS;
	shellInfo.lpVerb = "open";
	shellInfo.lpDirectory = program.c_str();
	shellInfo.nShow = 10;
	if (::ShellExecuteEx(&shellInfo))
	{ // success 
		hProcess = shellInfo.hProcess;
	} // success 
	return hProcess;
}



*/



/*
void Visualizerfebgeo::XMLwritestate(Vector time,Vector fx, Vector fy, Vector fz, Vector fox, Vector foy, Vector foz, Vector tx, Vector ty, Vector tz, Vector tox, Vector toy, Vector toz){
	ofstream opengeostate;
	INIReader ini = INIReader(INI_FILE);
	string resultDir1 = BASE_DIR + ini.Get("FEBIOSTEP", "GEO", "");
	string geof = ini.Get("FEBIOSTEP", "GEOF", "");
	string geos = ini.Get("FEBIOSTEP", "GEOS", "");
	string bd1 = ini.Get("BODYFORCES", "BDNAME1", "");
	string bd2 = ini.Get("BODYFORCES", "BDNAME2", "");
	//char itter = itteration + '0';
	////create a new folder for the analysis/////////
	int sizer = time.size();

	opengeostate = ofstream(resultDir1 + "/motionstate.mot", ofstream::out);
	opengeostate << "Coordinates" << endl;
	opengeostate << "version=1" << endl;
	opengeostate << "nRows=" <<sizer-1<< endl;
	opengeostate << "nColumns=13" << endl;
	opengeostate << "inDegrees=no" << endl;
	opengeostate << "" << endl;
	opengeostate << "Units are S.I.units(second, meters, Newtons, ...)" << endl;
	opengeostate << "Angles are not in degrees." << endl;
	opengeostate << "" << endl;
	opengeostate << "endheader" << endl;

	opengeostate << "  time  " << bd1 << "_tilt  " << bd1 << "_list  " << bd1 << "_rotation  " << bd1 << "_tx  " << bd1 << "_ty  " << bd1 << "_tz  " << bd2 << "_tilt  " << bd2 << "_list  " << bd2 << "_rotation  " << bd2 << "_tx  " << bd2 << "_ty  " << bd2 << "_tz  " << endl;
	for (int i = 0; i < sizer; ++i){
		opengeostate << time[i] << foz[i] << fox[i] << foy[i] << fz[i] << fx[i] << fy[i] << toz[i] << tox[i] << toy[i] << tz[i] << tx[i] << ty[i] << endl;

	}
}
*/