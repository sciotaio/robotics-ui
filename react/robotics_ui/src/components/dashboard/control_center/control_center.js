import React from "react";
import { useParams, useNavigate, useLocation } from "react-router-dom";
import { useState, useEffect, useContext, createContext, useMemo } from "react";
import queryString from "query-string";

import { Row, Col, Container, Button } from "react-bootstrap";
import { IoIosArrowDroprightCircle } from "react-icons/io";
import { MdOutlineSwipe } from "react-icons/md";

import { SideBar } from "../side_nav_bar/side_bar/side_bar";
import { Content } from "../content/desktop_layout/resizable_windows";
import { PhoneContent } from "../content/phone_layout/phone_layout";
import { BrowserView, MobileView, isBrowser, isMobile } from "react-device-detect";

const TopicContext = createContext();

export const ControlCenter = () => {
	
	const [component_list, setComponentList] = useState([]);
	const [preset_list, setPresetList] = useState([]);

	const location = useLocation();
	const navigate = useNavigate();

	function parseSettings(queryString) {
		const params = new URLSearchParams(queryString);
		/*// Log all parameters
		for (const [key, value] of params.entries()) {
			console.log(`${key}: ${value}`);
		}//*/

		const componentList = JSON.parse(params.get("component"));
		setComponentList(componentList);

		const presetListParam = params.get("preset_list");
		if (presetListParam) {
			try {
				const presetList = JSON.parse(presetListParam);
			setPresetList(presetList);
			} catch (error) {
				console.log("no preset given:");
			}			
		}
	}

	useEffect(() => {
		if (component_list.length === 0) {
			parseSettings(location.search);
		}
	}, []); // Empty dependency array means this effect runs once on mount
	
	useEffect(() => {
		if (component_list.length > 0 && preset_list.length > 0) {
        navigate({
            pathname: window.location.pathname,
            search: `?component=${JSON.stringify(component_list)}&preset_list=${JSON.stringify(preset_list)}`
        });
	}
    }, [component_list, preset_list]); // This effect runs whenever component_list or preset_list changes	

	const setCheckedStateById = (id, newState) => {
		setComponentList((prevList) => {
			return prevList.map((component) => {
				if (component.id === id) {
					return { ...component, state: newState };
				} else {
					return component;
				}
			});
		});
	};

	const setPositions = (id, newPosition) => {
		setComponentList((prevList) => {
			return prevList.map((component) => {
				if (component.id === id) {
					return { ...component, position: newPosition };
				} else {
					return component;
				}
			});
		});
	};

	const setSizes = (id, newSize) => {
		setComponentList((prevList) => {
			return prevList.map((component) => {
				if (component.id === id) {
					return { ...component, size: newSize };
				} else {
					return component;
				}
			});
		});
	};

	// resize and position Draggable and Resizable components

	const changeSizeAndPosition = (name, newSize, newPosition) => {
		setSizes(name, newSize);
		setPositions(name, newPosition);
	};

	const changeMultipleSizesAndPositions = (instructions) => {
		//console.log("Instructions: ", instructions);
		instructions.forEach((instruction) => {
			setCheckedStateById(instruction.id, true);
			changeSizeAndPosition(instruction.id, instruction.size, instruction.position);
		});
	};

	const [selectedPreset, setSelectedPreset] = useState(null);

	useEffect(() => {
		if (selectedPreset) {
			const preset = selectedPreset;

			const element = document.querySelector(".boundary-class");
			const rect = element.getBoundingClientRect();

			const max_with = Math.floor(rect.width);
			const max_height = Math.floor(rect.height);

			//console.log(`Max with: ${max_with}`);
			//console.log(`Max height: ${max_height}`);

			console.log(`Selected preset: ${JSON.stringify(preset, null, 2)}`);
			//changeMultipleSizesAndPositions(selectedPreset.settings);
			preset.settings.map((instruction) => {
				//console.log(`Instruction: ${JSON.stringify(instruction.id, null, 2)}`);

				setCheckedStateById(instruction.id, true);

				let position_x = Math.floor((instruction.position.x / preset.size_config.width) * max_with);
				let position_y = Math.floor(
					(instruction.position.y / preset.size_config.height) * max_height
				);
				let position_x_25 = position_x - (position_x % 25);
				let position_y_25 = position_y - (position_y % 25);

				let position = {
					x: position_x_25,
					y: position_y_25,
				};

				if (
					typeof instruction.size.width === "number" &&
					typeof instruction.size.height === "number"
				) {
					let size_width = Math.floor(
						(instruction.size.width / preset.size_config.width) * max_with
					);
					let size_height = Math.floor(
						(instruction.size.height / preset.size_config.height) * max_height
					);
					let size_with_25 = size_width - (size_width % 25);
					let size_height_25 = size_height - (size_height % 25);

					let size = {
						width: size_with_25,
						height: size_height_25,
					};

					//console.log(`Size: ${JSON.stringify(size, null, 2)}`);
					setSizes(instruction.id, size);
				}

				//console.log(`Position: ${JSON.stringify(position, null, 2)}`);
				setPositions(instruction.id, position);
			});
		}
	}, [selectedPreset]);

	useEffect(() => {
		//console.log(`component_list after: ${JSON.stringify(component_list, null, 2)}`);
	}, [component_list]);

	const [sideBarState, setSideBarState] = useState(true);
	const [disableSwipe, setDisableSwipe] = useState(false);

	const sharedStates = {
		component_list,
		setComponentList,
		preset_list,
		setPresetList,
		setPositions,
		setSizes,
		changeSizeAndPosition,
		changeMultipleSizesAndPositions,
		setCheckedStateById,
		selectedPreset,
		setSelectedPreset,
		sideBarState,
		setSideBarState,
		disableSwipe,
		setDisableSwipe,
	};

	return (
		<TopicContext.Provider value={sharedStates}>
			<Container fluid>
				<div className="z-1 position-absolute" style={{ padding: "10px 0px 0px 0px" }}>
					{!sideBarState && (
						<>
							<Button
								variant="primary"
								size="lg"
								block
								className="sidebar-button"
								onClick={() => setSideBarState(true)}
							>
								<IoIosArrowDroprightCircle size={40} color="black" backgroundColor="white" />
							</Button>
							<MobileView>
								<Button
									variant="primary"
									size="lg"
									block
									className="sidebar-button"
									onClick={() => setDisableSwipe(!disableSwipe)}
								>
									{disableSwipe ? (
										<MdOutlineSwipe size={40} color="red" backgroundColor="white" />
									) : (
										<MdOutlineSwipe size={40} color="black" backgroundColor="white" />
									)}
								</Button>
							</MobileView>
						</>
					)}
				</div>
				<div className="z-0 position-absolute" style={{ width: "98%", padding: "0px 0px 0px 0px" }}>
					<Row>
						{sideBarState && (
							<Col
								sm={2}
								style={{
									backgroundColor: "#111",
									color: "#fff",
									height: "100vh",
									width: "250px",
								}}
							>
								<div style={{ backgroundColor: "#111", height: "100vh" }}>
									<SideBar />
								</div>
							</Col>
						)}
						<Col style={{ height: "100vh", padding: "0px 0px 0px 0px" }}>
							<div
								className="boundary-class"
								style={{ position: "relative", height: "100vh", width: "100%" }}
							>
								<BrowserView>
									<Content />
								</BrowserView>
								<MobileView>
									<PhoneContent />
								</MobileView>
							</div>
						</Col>
					</Row>
				</div>
			</Container>
		</TopicContext.Provider>
	);
};

export function useTopicContext() {
	return useContext(TopicContext);
}
