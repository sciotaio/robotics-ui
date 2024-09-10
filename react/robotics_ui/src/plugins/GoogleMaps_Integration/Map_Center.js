"use client";

import { useState, useEffect } from "react";

import {
	APIProvider,
	Map,
	AdvancedMarker,
	Pin,
	MapControl,
	ControlPosition,
} from "@vis.gl/react-google-maps";

import { Polyline } from "./polyline";
import "./map_nav.css";

export const MapCenter = ({ topic }) => {
	const [data, setData] = useState([]);
	const [markerReached, setMarkerReached] = useState([]);

	const [clickCoordinates, setClickCoordinates] = useState([]);
	const [markers, setMarkers] = useState([]);
	const [center, setCenter] = useState({ lat: 51.03361194658677, lng: 13.779006180043893 });

	useEffect(() => {
		const fetchData = () => {
			
			// ==============================
			// GPS Position data of the robot
			// ==============================
			const route_1 = topic[0].route.toString();
			fetch(route_1)
				.then((response) => response.json())
				.then((data) => {
					setData(data);
			});
			console.log("Robot Position: \n", data);

			
			// ==============================
			// Signal that the robot reached the waypoints
			// ==============================
			const route = topic[1].route.toString();
			//console.log("Route: ", route);
			fetch(route)
				.then((response) => response.json())
				.then((data_1) => {
					setMarkerReached(data_1);
			});
			
		};

		fetchData(); // Fetch data immediately

		const intervalId = setInterval(fetchData, 5000); // Fetch data every 5 seconds

		// Clean up function
		return () => clearInterval(intervalId);
	}, [topic]);

	useEffect(() => {
		if (markerReached) {
			setMarkers(markers.filter((marker) => marker !== markerReached));
		}	
	}, [markerReached]);
	

	return (
		<APIProvider apiKey={process.env.REACT_APP_GOOGLE_MAPS_API_KEY}>
			<div style={{ height: "100%", width: "100%" }}>
				<Map
					mapId={process.env.REACT_APP_MAP_ID}
					onClick={(click) => {
						// console.log("click: ", click.detail.latLng);
						setClickCoordinates(click.detail.latLng);
						setMarkers([...markers, click.detail.latLng]);

						// ==============================
						// add waypoints to the route
						// ==============================
						const route = topic[2].route.toString();
						const body = { marker: click.detail.latLng };
						console.log(JSON.stringify(body, null, 2));
						fetch(route, {
							method: "POST",
							headers: {
								"Content-Type": "application/json",
							},
							body: JSON.stringify(body),
						})
							.then((response) => response.json())
							.then((data) => {
								console.log("Success:", data);
							})
							.catch((error) => {
								console.error("Error:", error);
							});
					}}
					onCenterChanged={(drag) => {
						setCenter(drag.detail.center);
					}}
					style={{ width: "100%", height: "100%" }}
					center={center}
					defaultZoom={18}
					gestureHandling={"greedy"}
					mapTypeId={"satellite"}
					reuseMaps={true}
				>
					<MapControl position={ControlPosition.TOP_LEFT}>
						<button
							className="map_nav_button"
							onClick={() => {
								if (
									data &&
									typeof data.latitude === "number" &&
									typeof data.longitude === "number"
								) {
									setCenter({ lat: data.latitude, lng: data.longitude });
								}
							}}
						>
							Center
						</button>
					</MapControl>
					{markers.map((marker, index) => (
						<AdvancedMarker
							key={index}
							position={marker}
							onClick={() => {
								setMarkers(markers.filter((_, markerIndex) => markerIndex !== index));
								
								// ==============================
								// delete waypoints from the route
								// ============================== 
								const route = topic[3].route.toString();
								const body = { marker: markers[index], index: index};
								console.log(JSON.stringify(body, null, 2));
								fetch(route, {
									method: "POST",
									headers: {
										"Content-Type": "application/json",
									},
									body: JSON.stringify(body),
								})
									.then((response) => response.json())
									.then((data) => {
										console.log("Success:", data);
									})
									.catch((error) => {
										console.error("Error:", error);
									});
							}}
						/>
					))}
					{markers.length > 0 && (
						<Polyline
							strokeWeight={4}
							strokeColor={"#ff22cc88"}
							path={[{ lat: data.latitude, lng: data.longitude }, ...markers]}
						/>
					)}
					{data && typeof data.latitude === "number" && typeof data.longitude === "number" && (
						<AdvancedMarker position={{ lat: data.latitude, lng: data.longitude }}>
							<Pin background={"#6497b1"} glyphColor={"#005b96"} borderColor={"#005b96"} />
						</AdvancedMarker>
					)}
				</Map>
			</div>
		</APIProvider>
	);
};
