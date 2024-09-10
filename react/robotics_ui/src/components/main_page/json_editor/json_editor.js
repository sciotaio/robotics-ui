// Install dependencies
// npm install react-bootstrap bootstrap

import React, { useEffect, useState } from "react";
import { Form, Button, Container, Row, Col, OverlayTrigger, Tooltip } from "react-bootstrap";
import "bootstrap/dist/css/bootstrap.min.css";

import { RenderType, RenderTypeTopicDefault } from "../../dashboard/topics/render_types";

export const JSONEditor = () => {
	const renderTypeKeys = Object.keys(RenderType);

	const defaultUrl =
		'http://localhost:3000/dashboard/settings?component=[{"id":"StringTopic","state":false,"render_type":"String","position":{"x":0,"y":0},"size":{"width":200,"height":200},"topic":[{"name":"topic1","topic":"/topic1","type":"String","route":"/subscriber/topic1"}]},{"id":"OpenStreetMap","state":false,"render_type":"OpenStreetMapCenter","position":{"x":0,"y":0},"size":{"width":650,"height":650},"topic":[{"name":"gps_cordinates","topic":"/gps/fix","type":"NavSatFix","route":"/subscriber/gps_cordinates"},{"name":"waypoint_reached","topic":"/waypoint_reached","type":"String","route":"/subscriber/waypoint_reached"},{"name":"waypoint_add","topic":"/waypoint_add","type":"String","route":"/publisher/waypoint_add"},{"name":"waypoint_delete","topic":"/waypoint_delete","type":"String","route":"/publisher/waypoint_delete"}]}]&preset_list=undefined';

	const [formData, setFormData] = useState({
		component: [],
		preset_list: [],
	});

	const [url, setUrl] = useState("");
	const [generatedUrl, setGeneratedUrl] = useState("");
	useEffect(() => {
		const currentPageUrl = window.location.href;
		const params = `${currentPageUrl}dashboard/settings?component=${JSON.stringify(
			formData.component
		)}&preset_list=${JSON.stringify(formData.preset)}`;
		setGeneratedUrl(params);
	}, [formData]);

	const handelClear = () => {
		setFormData({
			component: [],
			preset_list: [],
		});
	};

	const handleInputChange = (e, section, index, subIndex) => {
		const { name, value } = e.target;
		const updatedFormData = { ...formData };

		if (section === "component") {
			updatedFormData.component[index][name] = value;
		} else if (section === "preset_list") {
			updatedFormData.preset_list[index][name] = value;
		} else if (section === "topic") {
			updatedFormData.component[index].topic[subIndex][name] = value;
		} else if (section === "component_size") {
			updatedFormData.component[index].size[name] = value;
		} else if (section === "component_position") {
			updatedFormData.component[index].position[name] = value;
		}

		setFormData(updatedFormData);
	};

	const handleRenderTypeChange = (e, section, index, subIndex) => {
		handleInputChange(e, "component", index);
		const render_type = e.target.value;
		const validRenderTypes = Object.keys(RenderTypeTopicDefault);

		if (validRenderTypes.includes(render_type)) {
			const render_type_topic = RenderTypeTopicDefault[render_type];
			const updatedComponents = [...formData.component];
			updatedComponents[index].topic = [];
			render_type_topic.topic.map((topic, subIndex) => {
				console.log("topic", topic);
				updatedComponents[index].topic.push(topic);
			});
			setFormData({ ...formData, component: updatedComponents });
		} else {
			console.log("Invalid Render Type");
		}
	};

	const handleAddComponent = () => {
		setFormData({
			...formData,
			component: [
				...formData.component,
				{
					id: "",
					state: false,
					render_type: "",
					position: { x: 0, y: 0 },
					size: { width: 300, height: 300 },
					topic: [],
				},
			],
		});
	};

	const handleDeleteComponent = (index) => {
		const updatedComponents = [...formData.component];
		updatedComponents.splice(index, 1);
		setFormData({ ...formData, component: updatedComponents });
	};

	const handleAddTopic = (index) => {
		const updatedComponents = [...formData.component];
		updatedComponents[index].topic.push({
			name: "",
			topic: "",
			render_segment: "",
			type: "",
			route: "",
		});
		setFormData({ ...formData, component: updatedComponents });
	};

	const handleDeleteTopic = (index, subIndex) => {
		const updatedComponents = [...formData.component];
		updatedComponents[index].topic.splice(subIndex, 1);
		setFormData({ ...formData, component: updatedComponents });
	};

	const handleAddPreset = () => {
		setFormData({
			...formData,
			preset_list: [
				...formData.preset_list,
				{
					preset_name: "",
					size_config: { width: 0, height: 0 },
					settings: [],
				},
			],
		});
	};

	const handleFileUpload = (e) => {
		const file = e.target.files[0];
		const reader = new FileReader();
		reader.onload = (event) => {
			try {
				const json = JSON.parse(event.target.result);
				setFormData(json);
			} catch (error) {
				console.error("Error parsing JSON:", error);
			}
		};
		reader.readAsText(file);
	};

	const extractJsonFromUrl = (url) => {
		const urlObj = new URL(url);
		const params = new URLSearchParams(urlObj.search);

		const componentList = params.get("component");
		const presetList = params.get("preset_list");
		let decodedComponentList = [];
		let decodedPresetList = [];

		try {
			decodedComponentList = JSON.parse(decodeURIComponent(componentList));
		} catch (error) {
			try {
				decodedComponentList = JSON.parse(componentList);
			} catch (error) {
				console.log("no componentList given:");
			}
		}

		if (presetList) {
			try {
				decodedPresetList = JSON.parse(decodeURIComponent(presetList));
			} catch (error) {
				console.log("no preset given:");
			}
		}

		return {
			componentList: decodedComponentList,
			presetList: decodedPresetList,
		};
	};

	const handelLinkPaste = (e) => {
		setUrl(e.target.value);
	};

	const handelLinkUpload = (type) => {
		console.log("url", url);
		let useUrl = url;
		if (type === "default_link") {
			useUrl = defaultUrl;
		}
		console.log("useUrl", useUrl);
		const { componentList, presetList } = extractJsonFromUrl(useUrl);
		setFormData({
			component: componentList,
			preset_list: presetList,
		});
	};

	const generateJsonData = () => {
		return JSON.stringify(formData, null, 2);
	};

	const downloadJson = () => {
		const jsonData = generateJsonData();
		const blob = new Blob([jsonData], { type: "application/json" });
		const url = URL.createObjectURL(blob);
		const link = document.createElement("a");
		link.href = url;
		link.download = "topic_list.json";
		document.body.appendChild(link);
		link.click();
		document.body.removeChild(link);
	};

	const handleSubmit = (e) => {
		e.preventDefault();
		console.log(JSON.stringify(formData, null, 2));
	};

	return (
		<Container>
			<h1>JSON Editor</h1>
			<Form onSubmit={handleSubmit}>
				<Form.Group>
					<Form.Label>Upload JSON File</Form.Label>
					<Form.Control type="file" accept=".json" onChange={handleFileUpload} />
				</Form.Group>
				<br />
				<Form.Group>
					<Form.Label>Upload Link</Form.Label>
					<Form.Control type="text" accept=".json" onChange={handelLinkPaste} />
				</Form.Group>
				<br />
				<Button variant="secondary" onClick={() => handelLinkUpload("link")}>
					Apply Link JSON
				</Button>
				<Button
					style={{ marginLeft: "10px" }}
					variant="secondary"
					onClick={() => handelLinkUpload("default_link")}
				>
					Apply Default Link JSON
				</Button>
				<br />
				<br />
				<Button variant="secondary" onClick={() => handelClear()}>
					Clear JSON
				</Button>
				<br />
				<br />
				<hr />
				{formData.component.map((component, index) => (
					<div key={index}>
						<h4>Component {index + 1}</h4>
						<Form.Group as={Row}>
							<OverlayTrigger
								placement="top"
								overlay={
									<Tooltip id={`tooltip-id`}>
										Name of your component (has to be uinque){" "}
									</Tooltip>
								}
							>
								<Form.Label column sm="2">
									ID
								</Form.Label>
							</OverlayTrigger>
							<Col sm="10">
								<Form.Control
									type="text"
									name="id"
									value={component.id}
									onChange={(e) => handleInputChange(e, "component", index)}
								/>
							</Col>
						</Form.Group>
						<Form.Group as={Row}>
							<OverlayTrigger
								placement="top"
								overlay={
									<Tooltip id={`tooltip-id`}>Visibility of your component on load</Tooltip>
								}
							>
								<Form.Label column sm="2">
									Visibility (State)
								</Form.Label>
							</OverlayTrigger>
							<Col sm="10">
								<Form.Select
									aria-label="Default select example"
									name="state"
									value={component.state}
									defaultValue={"false"}
									onChange={(e) => handleRenderTypeChange(e, "component", index)}
								>
									<option>false</option>
									<option>true</option>
								</Form.Select>
							</Col>
						</Form.Group>
						<Form.Group as={Row}>
							<OverlayTrigger
								placement="top"
								overlay={
									<Tooltip id={`tooltip-id`}>
										Desides what plugin your component uses
									</Tooltip>
								}
							>
								<Form.Label column sm="2">
									Render Type
								</Form.Label>
							</OverlayTrigger>
							<Col sm="10">
								<Form.Select
									aria-label="Default select example"
									name="render_type"
									value={component.render_type}
									defaultValue={"Generic"}
									onChange={(e) => handleRenderTypeChange(e, "component", index)}
								>
									<option>Select Render Type</option>
									{renderTypeKeys.map((key) => (
										<option key={key} value={key}>
											{key}
										</option>
									))}
								</Form.Select>
							</Col>
						</Form.Group>
						<Form.Group as={Row}>
							<Form.Label column sm="2">
								Position x
							</Form.Label>
							<Col sm="10">
								<Form.Control
									type="text"
									name="x"
									value={component.position.x}
									onChange={(e) => handleInputChange(e, "component_position", index)}
								/>
							</Col>
						</Form.Group>
						<Form.Group as={Row}>
							<Form.Label column sm="2">
								Position Y
							</Form.Label>
							<Col sm="10">
								<Form.Control
									type="text"
									name="y"
									value={component.position.y}
									onChange={(e) => handleInputChange(e, "component_position", index)}
								/>
							</Col>
						</Form.Group>
						<Form.Group as={Row}>
							<Form.Label column sm="2">
								Size width
							</Form.Label>
							<Col sm="10">
								<Form.Control
									type="text"
									name="width"
									value={component.size.width}
									onChange={(e) => handleInputChange(e, "component_size", index)}
								/>
							</Col>
						</Form.Group>
						<Form.Group as={Row}>
							<Form.Label column sm="2">
								Size height
							</Form.Label>
							<Col sm="10">
								<Form.Control
									type="text"
									name="height"
									value={component.size.height}
									onChange={(e) => handleInputChange(e, "component_size", index)}
								/>
							</Col>
						</Form.Group>
						<Button variant="success" onClick={() => handleAddTopic(index)}>
							Add Topic
						</Button>
						<Button variant="danger" onClick={() => handleDeleteComponent(index)}>
							Delete Component
						</Button>
						{component.topic.map((topic, subIndex) => (
							<div key={subIndex} style={{ marginLeft: "20px" }}>
								<OverlayTrigger
									placement="top-start"
									overlay={
										<Tooltip id={`tooltip-id`}>
											Ros2 Topics you want to pass to your component (Plese consult the
											plugin manual){" "}
										</Tooltip>
									}
								>
									<h5>Topic {subIndex + 1}</h5>
								</OverlayTrigger>
								<Form.Group as={Row}>
									<OverlayTrigger
										placement="top"
										overlay={<Tooltip id={`tooltip-id`}>Name of your Topic</Tooltip>}
									>
										<Form.Label column sm="2">
											Name
										</Form.Label>
									</OverlayTrigger>
									<Col sm="10">
										<Form.Control
											type="text"
											name="name"
											value={topic.name}
											onChange={(e) => handleInputChange(e, "topic", index, subIndex)}
										/>
									</Col>
								</Form.Group>
								<Form.Group as={Row}>
									<OverlayTrigger
										placement="top"
										overlay={
											<Tooltip id={`tooltip-id`}>Name of your Topic in Ros</Tooltip>
										}
									>
										<Form.Label column sm="2">
											Topic
										</Form.Label>
									</OverlayTrigger>
									<Col sm="10">
										<Form.Control
											type="text"
											name="topic"
											value={topic.topic}
											onChange={(e) => handleInputChange(e, "topic", index, subIndex)}
										/>
									</Col>
								</Form.Group>
								<Form.Group as={Row}>
									<OverlayTrigger
										placement="top"
										overlay={
											<Tooltip id={`tooltip-id`}>
												Ros 2 Mesage Type of your Topic
											</Tooltip>
										}
									>
										<Form.Label column sm="2">
											Type
										</Form.Label>
									</OverlayTrigger>
									<Col sm="10">
										<Form.Control
											type="text"
											name="type"
											value={topic.type}
											onChange={(e) => handleInputChange(e, "topic", index, subIndex)}
										/>
									</Col>
								</Form.Group>
								<Form.Group as={Row}>
									<OverlayTrigger
										placement="top"
										overlay={
											<Tooltip id={`tooltip-id`}>Route to acces the Backend</Tooltip>
										}
									>
										<Form.Label column sm="2">
											Route
										</Form.Label>
									</OverlayTrigger>
									<Col sm="10">
										<Form.Control
											type="text"
											name="route"
											value={topic.route}
											onChange={(e) => handleInputChange(e, "topic", index, subIndex)}
										/>
									</Col>
								</Form.Group>
								<Button variant="danger" onClick={() => handleDeleteTopic(index, subIndex)}>
									Delete Topic
								</Button>
							</div>
						))}
					</div>
				))}
				<br />
				<Button variant="success" onClick={handleAddComponent}>
					Add Component
				</Button>
				<br />
				<hr />
				<Button variant="primary" type="submit">
					Generate JSON
				</Button>
				<Button style={{ marginLeft: "10px" }} variant="primary" onClick={() => downloadJson()}>
					Download JSON
				</Button>
				<br />
				<br />
				<Form.Group>
					<Form.Label>Link</Form.Label>
					<Form.Control type="text" value={generatedUrl} />
				</Form.Group>
				<Button style={{ marginLeft: "0px" }} variant="primary" href={generatedUrl}>
					Use JSON
				</Button>
			</Form>
		</Container>
	);
};
