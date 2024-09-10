import { useTopicContext } from "../../control_center/control_center";
import React from "react";
import { useState, useEffect } from "react";
import SwipeableViews from "react-swipeable-views-react-18-fix";

import "./phone_layout.css";

import { Topics } from "../../topics/topics";
import { Button } from "react-bootstrap";

export const PhoneContent = ({}) => {
	const { component_list, disableSwipe, setDisableSwipe } =
		useTopicContext();

    const [selectedComponents, setSelectedComponents] = useState([]);

    useEffect(() => {
		if (component_list) {
			setSelectedComponents(component_list.filter((component) => component.state === true));
		}
	}, [component_list]);
	
    useEffect(() => {
        console.log("Diable Swipe: ", disableSwipe);
    }, [disableSwipe]);
    
    return (
		<div >
			<SwipeableViews
                disabled = {disableSwipe}
            >
				{selectedComponents.map((component, index) => {
                    if (component.state) {
                        //console.log("Component: ", component.topic)
                        const topic = component.topic;
                        return (
                            <>
                                <div className="content_box">
                                    <div className="content_header">
                                        topic: {component.id}
                                    </div>
                                    <div 
                                        className="content_body"
                                    >
                                        <Topics component={component} />
                                    </div>
                                    <div className="content_footer">
                                        
                                    </div>
                                </div>
                            </>
                        );
                    }
                })}
			</SwipeableViews>
		</div>
	);
};
