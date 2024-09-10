"use client";

import { useState, useEffect } from "react";
import { MapContainer, TileLayer, Marker, Popup, useMapEvents, Polyline, useMap } from "react-leaflet";
import { Icon } from "leaflet";
import Control from "react-leaflet-custom-control";

import { useTopicContext } from "../../components/dashboard/control_center/control_center";

import "leaflet/dist/leaflet.css";
import "./map_nav.css";

// icon source: https://www.flaticon.com/authors/kmg-design
// leflet: https://react-leaflet.js.org/

export const OpenStreetMapCenter = ({ topic }) => {
	const { component_list } = useTopicContext();
	
	// test without gazebo backend
	const [robotPosition, setRobotPosition] = useState({ latitude: 51.03361194658677, longitude: 13.779006180043893 });
		
	//const [robotPosition, setRobotPosition] = useState([]);
	const [markerReached, setMarkerReached] = useState([]);
	const [markers, setMarkers] = useState([]);
	const [center, setCenter] = useState({ lat: 51.03361194658677, lng: 13.779006180043893 });
	const [zoom, setZoom] = useState(18);

	useEffect(() => {
		const fetchData = () => {
			// ==============================
			// GPS Position data of the robot
			// ==============================
			const route_1 = topic[0].route.toString();
			fetch(route_1)
				.then((response) => response.json())
				.then((data) => {
					if (data != null) {
						setRobotPosition(data);
					}
					
				});
			console.log("Robot Position: \n", robotPosition);

			// ==============================
			// Signal that the robot reached the waypoints
			// ==============================
			const route = topic[1].route.toString();
			fetch(route)
				.then((response) => response.json())
				.then((data_1) => {
					if (data_1 != null)
						setMarkerReached(JSON.parse(data_1.data));
						//console.log("data: Marker Reached: \n", JSON.parse(data_1.data));
				});

		};

		fetchData(); // Fetch data immediately

		const intervalId = setInterval(fetchData, 5000); // Fetch data every 5 seconds

		// Clean up function
		return () => clearInterval(intervalId);
	}, [topic]);

	useEffect(() => {
		if (markerReached) {
			console.log("Marker Reached useEffect: ", markerReached);
			console.log("Markers: ", markers);
			markers.forEach(marker => {
				if (marker.lat === markerReached.marker.lat && marker.lng === markerReached.marker.lng) {
					console.log(`Marker Reached lat: ${markerReached.marker.lat}, lng: ${markerReached.marker.lng}`);
					setMarkers(markers.filter((marker_o) => marker_o !== marker));
				}
			});
		}
	}, [markerReached]);

	const Markers = () => {
		const map = useMapEvents({
			click(e) {
				const clickLocation = { lat: e.latlng.lat, lng: e.latlng.lng };
				console.log("Clicked Location: ", clickLocation);
				setMarkers([...markers, clickLocation]);

				// ==============================
				// add waypoints to the route
				// ==============================
				const route = topic[2].route.toString();
				const body = { marker: clickLocation };
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
			},
		});

		return (
			<>
				{markers.map((marker, index) => (
					<Marker
						key={index}
						position={marker}
						icon={waypointIcon}
						eventHandlers={{
							click: (e) => {
								setMarkers(markers.filter((_, markerIndex) => markerIndex !== index));

								// ==============================
								// delete waypoints from the route
								// ==============================
								const route = topic[3].route.toString();
								const body = { marker: markers[index], index: index };
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
							},
						}}
					/>
				))}
				{robotPosition &&
					typeof robotPosition.latitude === "number" &&
					typeof robotPosition.longitude === "number" && (
						<Marker
							position={{ lat: robotPosition.latitude, lng: robotPosition.longitude }}
							icon={robotIcon}
						/>
					)}
				{markers.length > 0 && robotPosition &&
					typeof robotPosition.latitude === "number" &&
					typeof robotPosition.longitude === "number" &&(
					<Polyline
						pathOptions={{ color: "#ff22cc88" }}
						positions={[
							{ lat: robotPosition.latitude, lng: robotPosition.longitude },
							...markers,
						]}
					/>
				)}
			</>
		);
	};

	const waypointIcon = new Icon({
		iconUrl: require("./Icon/marker_red.png"),
		iconSize: [25, 25],
	});

	const robotIcon = new Icon({
		iconUrl: require("./Icon/marker_blue.png"),
		iconSize: [25, 25],
	});

	function ChangeView({ center, zoom }) {
		const map = useMap();
		map.setView(center, zoom);
		return null;
	}

	function UpdateView() {
		const map = useMap();
		map.addEventListener("drag", () => {
			setCenter(map.getCenter());
		});
		map.addEventListener("zoom", () => {
			setZoom(map.getZoom());
			setCenter(map.getCenter());
		});
	}

	function UpdateMap() {
		const map = useMap();
		 useEffect(() => {
			map.invalidateSize()
		 }, [component_list])
	}

	return (
		<div style={{ height: "100%", width: "100%" }}>
			<MapContainer center={center} zoom={zoom} maxZoom={40}>
				<ChangeView center={center} zoom={zoom} />
				<UpdateView />
				<UpdateMap />
				<TileLayer
					attribution=' <a href="https://www.openstreetmap.org/copyright"> OpenStreetMap <a href="https://www.flaticon.com/free-icons/marker"> Flaticon'
					url="https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png"
					maxZoom={40}
				/>

				<Markers />
				<Control prepend position="topright">
					<button
						className="map_nav_button"
						onClick={() => {
							if (
								robotPosition &&
								typeof robotPosition.latitude === "number" &&
								typeof robotPosition.longitude === "number"
							) {
								setCenter({ lat: robotPosition.latitude, lng: robotPosition.longitude });
								setZoom(18);
							}
						}}
					>
						Center
					</button>
				</Control>
			</MapContainer>
		</div>
	);
};
