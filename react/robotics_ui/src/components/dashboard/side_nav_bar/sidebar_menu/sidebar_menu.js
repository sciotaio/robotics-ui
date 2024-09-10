import { ListGroup, Form } from "react-bootstrap";
import { useState } from "react";

import { useTopicContext } from "../../control_center/control_center";

export const SidebarMenu = ({}) => {
    
    const { component_list, setCheckedStateById, setSelectedPreset } = useTopicContext();

    const handleCheckboxChange = (componentId, currentState) => {
        console.log("Checkbox chnage: ", componentId);
        setCheckedStateById(componentId, !currentState);
        setSelectedPreset(null);
    };

    const createCheckboxes = () => {
        return component_list.map((component, index) => (
            <ListGroup.Item key={index} onClick={() => setCheckedStateById(component.id, !component.state)}>
                <label style={{ width: "100%", display: "block" }}>
                    <Form.Check 
                        type="checkbox" 
                        label={component.id} 
                        checked={component.state} 
                        onChange={() => handleCheckboxChange(component.id, component.state)}
                    />
                </label>
            </ListGroup.Item>
        ));
    };

    return (
        <div style={{ textAlign: "left" }}>
            <ListGroup>
                {createCheckboxes(component_list)}
            </ListGroup>
        </div>
    );
};