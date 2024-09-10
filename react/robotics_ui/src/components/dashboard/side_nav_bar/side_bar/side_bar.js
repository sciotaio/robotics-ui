import {
	Button,
	Card,
	Stack,
	Nav,
	Table,
	Collapse,
	Dropdown,
	Tooltip,
	OverlayTrigger,
} from "react-bootstrap";
import { Row, Col, Container } from "react-bootstrap";
import { useState } from "react";

import { SidebarMenu } from "../sidebar_menu/sidebar_menu";

import { useTopicContext } from "../../control_center/control_center";
import "./sidebar.css";
import { BsArrowUp, BsArrowDown } from "react-icons/bs";
import { PresetDropdown } from "../preset_dropdown/preset_dropdown";

export const SideBar = ({}) => {
	const {
		changeMultipleSizesAndPositions,
		component_list,
		setPresetList,
		preset_list,
		sideBarState,
		setSideBarState,
	} = useTopicContext();

	const [open, setOpen] = useState(true);
	const [inputValue, setInputValue] = useState("");

	const handleInputChange = (event) => {
		setInputValue(event.target.value);
	};

	const createPresetFromCurrentSettings = (preset_title) => {
		// Get the current settings from the component_list
		const currentSettings = component_list
			.filter((component) => component.state === true)
			.map((component) => ({
				id: component.id,
				size: component.size,
				position: component.position,
			}));

		const element = document.querySelector(".boundary-class");
		const rect = element.getBoundingClientRect();
		const max_with = Math.floor(rect.width) - (Math.floor(rect.width) % 25);
		const max_height = Math.floor(rect.height) - (Math.floor(rect.height) % 25);

		// Create a new preset with these settings
		const newPreset = {
			preset_name: preset_title, // Give your new preset a name
			size_config: { width: max_with, height: max_height },
			settings: currentSettings,
		};

		// Add the new preset to the preset_list
		setPresetList([...preset_list, newPreset]);
	};

	return (
		<Table striped bordered hover className="sidebar-table">
			<thead>
				<tr>
					<th colSpan={2}>
						<OverlayTrigger
							placement="right"
							overlay={<Tooltip id="tooltip-home" className="custom-tooltip">Go to Home</Tooltip>}
						>
							<Button
								variant="primary"
								size="lg"
								block
								className="close-sidebar-button"
								href="/"
							>
								Robotics UI
							</Button>
						</OverlayTrigger>
					</th>
				</tr>
			</thead>
			<tbody>
				<tr>
					<td colSpan={2}>
						<Button
							variant="primary"
							size="lg"
							block
							className="sidebar-button"
							onClick={() => setSideBarState(false)}
						>
							Collapse SideBar
						</Button>
					</td>
				</tr>
				<tr>
					<td colSpan={2}>
						<PresetDropdown />
					</td>
				</tr>
				<tr>
					<td colSpan={1} style={{ verticalAlign: "middle" }}>
						<input
							type="text"
							value={inputValue}
							onChange={handleInputChange}
							style={{ width: "100%" }}
						/>
					</td>
					<td>
					<OverlayTrigger
							placement="right"
							overlay={<Tooltip id="tooltip-preset" className="custom-tooltip">Create new Preset</Tooltip>}
						>
						<Button
							variant="primary"
							size="lg"
							block
							className="sidebar-button"
							onClick={() => createPresetFromCurrentSettings(inputValue)}
						>
							New
						</Button>
						</OverlayTrigger>
					</td>
				</tr>
				<tr>
					<td colSpan={1}>
						<Button
							variant="primary"
							size="lg"
							block
							className="sidebar-button"
							onClick={() => setOpen(!open)}
						>
							Menu
						</Button>
					</td>
					<td>
						<Button
							variant="primary"
							size="lg"
							block
							className="sidebar-button"
							onClick={() => setOpen(!open)}
						>
							{open ? (
								<BsArrowUp className="arrow-up" size={25} color="white" />
							) : (
								<BsArrowDown className="arrow-down" size={25} color="white" />
							)}
						</Button>
					</td>
				</tr>
				<tr>
					<td colSpan={2}>
						<Collapse in={open}>
							<div>
								<SidebarMenu />
							</div>
						</Collapse>
					</td>
				</tr>
			</tbody>
		</Table>
	);
};
