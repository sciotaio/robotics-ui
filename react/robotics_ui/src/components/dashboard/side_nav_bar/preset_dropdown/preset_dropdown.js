import { Dropdown } from "react-bootstrap";
import { useTopicContext } from "../../control_center/control_center";
import { useState, useEffect } from "react";

export const PresetDropdown = () => {
	const {
		preset_list,
		changeMultipleSizesAndPositions,
		changeSizeAndPosition,
		component_list,
		setSizes,
		setPositions,
        setCheckedStateById,
        setSelectedPreset
	} = useTopicContext();
	
    

	return (
		<Dropdown style={{ width: "100%" }}>
			<Dropdown.Toggle
				variant="success"
				id="dropdown-basic"
				style={{ backgroundColor: "transparent", border: "none" }}
			>
				Chose Preset
			</Dropdown.Toggle>

			<Dropdown.Menu >
				{preset_list.map((preset, index) => (
					<Dropdown.Item
						key={index}
						onClick={() => {
							setSelectedPreset(preset);
						}}
					>
						{preset.preset_name}
					</Dropdown.Item>
				))}
			</Dropdown.Menu>
		</Dropdown>
	);
};
