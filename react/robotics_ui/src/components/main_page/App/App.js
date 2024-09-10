import { ControlCenter } from "../../dashboard/control_center/control_center";
import { Home } from "../home/home";

import { BrowserRouter as Router, Route, Routes, Navigate } from "react-router-dom";
import { useState, useMemo, useEffect } from "react";

function App() {
	const json_data = useMemo(() => {
		return {
			component: [
				{
					id: "topic1",
					state: false,
					render_type: "Map",
					position: { x: 0, y: 0 },
					size: { width: 200, height: 200 },
					topic: [
						{
							name: "topic1",
							topic: "/topic1",
							render_segment: "GPS",
							type: "String",
							route: "/subscriber/topic1",
						},

						{
							name: "topic2",
							topic: "/topic1",
							render_segment: "GPS",
							type: "String",
							route: "/subscriber/topic1",
						},
					],
				},
				{
					id: "topic2",
					state: false,
					render_type: "String",
					position: { x: 0, y: 0 },
					size: { width: 200, height: 200 },
					topic: [
						{
							name: "topic1",
							topic: "/topic1",
							type: "String",
							route: "/subscriber/topic1",
						},
					],
				},
				{
					id: "topic3",
					state: false,
					render_type: "test",
					position: { x: 0, y: 0 },
					size: { width: 200, height: 200 },
					topic: [
						{
							name: "topic1",
							topic: "/topic1",
							type: "String",
							route: "/subscriber/topic1",
						},
					],
				},
				{
					id: "Map",
					state: false,
					render_type: "MapCenter",
					position: { x: 0, y: 0 },
					size: { width: 650, height: 650 },
					topic: [
						{
							name: "gps_cordinates",
							topic: "/gps/fix",
							type: "NavSatFix",
							route: "/subscriber/gps_cordinates",
						},
						{
							name: "waypoint_reached",
							topic: "/waypoint_reached",
							type: "String",
							route: "/subscriber/waypoint_reached",
						},
						{
							name: "waypoint_add",
							topic: "/waypoint_add",
							type: "String",
							route: "/publisher/waypoint_add",
						},
						{
							name: "waypoint_delete",
							topic: "/waypoint_delete",
							type: "String",
							route: "/publisher/waypoint_delete",
						},
					],
				},
				{
					id: "OpenStreetMap",
					state: false,
					render_type: "OpenStreetMapCenter",
					position: { x: 0, y: 0 },
					size: { width: 650, height: 650 },
					topic: [
						{
							name: "gps_cordinates",
							topic: "/gps/fix",
							type: "NavSatFix",
							route: "/subscriber/gps_cordinates",
						},
						{
							name: "waypoint_reached",
							topic: "/waypoint_reached",
							type: "String",
							route: "/subscriber/waypoint_reached",
						},
						{
							name: "waypoint_add",
							topic: "/waypoint_add",
							type: "String",
							route: "/publisher/waypoint_add",
						},
						{
							name: "waypoint_delete",
							topic: "/waypoint_delete",
							type: "String",
							route: "/publisher/waypoint_delete",
						},
					],
				},
				{
					id: "topic5",
					state: false,
					render_type: "test",
					position: { x: 0, y: 0 },
					size: { width: 200, height: 200 },
					topic: [
						{
							name: "gps_cordinates",
							topic: "/gps/fix",
							type: "NavSatFix",
							route: "/subscriber/gps_cordinates",
						},
					],
				},
			],
			preset_list: [
				{
					preset_name: "preset1",
					size_config: { width: 1625, height: 900 },
					settings: [
						{
							id: "OpenStreetMap",
							size: { width: 1200, height: 900 },
							position: { x: 0, y: 0 },
						},
						{
							id: "topic2",
							size: { width: 350, height: 300 },
							position: { x: 1200, y: 0 },
						},
						{
							id: "topic3",
							size: { width: 350, height: 300 },
							position: { x: 1200, y: 300 },
						},
						{
							id: "topic1",
							size: { width: 350, height: 300 },
							position: { x: 1200, y: 600 },
						},
					],
				},
			],
		};
	}, []);

	// Load default component_list and preset_list
	const [component_list, setComponentList] = useState([]);
	const [preset_list, setPresetList] = useState([]);
	const [settings, setSettings] = useState({});
	const [isSettingsBuilt, setIsSettingsBuilt] = useState(false);

	useEffect(() => {
		setPresetList(json_data.preset_list);
		setComponentList(json_data.component);
	}, []); // Empty dependency array means this effect runs once on mount

	useEffect(() => {
		if (component_list.length > 0 && preset_list.length > 0) {
			setSettings(
				`?component=${JSON.stringify(component_list)}&preset_list=${JSON.stringify(preset_list)}`
			);
			setIsSettingsBuilt(true);
		}
		
	}, [component_list, preset_list]);

	return (
		<Router>
			<Routes>
				<Route
					path="/"
					element={
						<div>
							<Home />
						</div>
					}
				/>
				<Route
					path="dashboard"
					element={
						<>
							{isSettingsBuilt && <Navigate to={`/dashboard/settings${settings}`} />}
						</>
					}
				/>
				<Route
					path="dashboard/:settings"
					element={
						<div
							style={{
								height: "100vh",
								position: "absolute",
								width: "100%",
								overflow: "hidden",
								padding: "0px 0px 0px 0px",
							}}
						>
							<ControlCenter />
						</div>
					}
				/>
			</Routes>
		</Router>
	);
}

export default App;
