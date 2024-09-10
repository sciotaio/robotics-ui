import Draggable from "react-draggable";
import { Resizable } from "re-resizable";

import { useTopicContext } from "../../control_center/control_center";
import "./dragable.css";
import "react-resizable/css/styles.css";

import { Topics } from "../../topics/topics";

export const Content = ({}) => {
	const { component_list, changeSizeAndPosition, setPositions, setSizes, setSelectedPreset } =
		useTopicContext();

	const handleStart = (e, data) => {
		console.log("Event: ", e);
		console.log("Data: ", data);
	};

	const dragHandle = (component, e, data) => {
		console.log("Event: ", e);
		console.log("Data: ", data);
		console.log("Position: ", data.x, data.y);
		setPositions( component.id, { x: data.x, y: data.y } );
		setSelectedPreset(null);
	};

	const reszieHandle = (e, data) => {
		//console.log("Event: ", e);
		//console.log("Data: ", data);
	};

	const dragHandleStop = (e, data) => {
		//console.log("Event: ", e);
		//console.log("Data: ", data);
	};

	const resizeHandleStop = (component, e, direction, refToElement) => {
		//console.log("Event: ", e);
		//console.log("Direction: ", direction);
		//console.log("New width: ", refToElement.offsetWidth);
		//console.log("New height: ", refToElement.offsetHeight);
		setSizes( component.id, { width: refToElement.offsetWidth, height: refToElement.offsetHeight } );
		setSelectedPreset(null);
	};

	const createDraggables = () => {
		return component_list.map((component, index) => {
			if (component.state) {
				//console.log("Component: ", component.topic)
				const topic = component.topic;
				return (
					<Draggable
						handle="strong"
						position={component.position || { x: 0, y: 0 }}
						grid={[25, 25]}
						scale={1}
						onStart={handleStart}
						onDrag={(e, data) => dragHandle(component, e, data)}
						onStop={dragHandleStop}
						bounds=".boundary-class"
						key={`${component.id}-${component.state}`}
					>
						<Resizable
							size={component.size || { width: 200, height: 200 }}
							onResizeStart={handleStart}
							onResize={reszieHandle}
							onResizeStop={(e, direction, refToElement) =>
								resizeHandleStop(component, e, direction, refToElement)
							}
							grid={[25, 25]}
							style={{ position: "absolute" }}
						>
							<div className="box no-cursor" style={{ position: "absolute" }}>
								<strong className="cursor">
									<div>
										topic: {component.id} ({component.size?.width || 200} X{" "}
										{component.size?.height || 200} )
									</div>
								</strong>
								<Topics component={component} />
							</div>
						</Resizable>
					</Draggable>
				);
			}
			return null;
		});
	};

	return <>{createDraggables()}</>;
};
