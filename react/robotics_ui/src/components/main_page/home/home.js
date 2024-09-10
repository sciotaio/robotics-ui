import React from "react";
import Nav from "react-bootstrap/Nav";
import Navbar from "react-bootstrap/Navbar";
import { useState } from "react";

import { Button } from "react-bootstrap";

import { JSONEditor } from "../json_editor/json_editor";

import "./home.css";

export const Home = ({}) => {

	const [navBarOption, setNavBarOption] = useState("");

	const handelNavBarOption = (option) => {
		setNavBarOption(option);
	}

	return (
		<div>
			<Navbar bg="dark" data-bs-theme="dark" className="navbar">
				<Navbar.Brand href="/" className="navbar-title" >Robotics UI</Navbar.Brand>
				<Navbar.Toggle aria-controls="basic-navbar-nav" />
				<Navbar.Collapse id="basic-navbar-nav">
					<Button className="nav-button" onClick={() => handelNavBarOption("JsonEditor")}>
						JSON Editor
					</Button>
					<Nav className="mr-auto">
						<Nav.Link className="nav-link" href="/dashboard">
							Dashboard
						</Nav.Link>
					</Nav>
					<Button variant="outline-success" onClick={() => handelNavBarOption("Login")} className="nav-button2">
						Login
					</Button>
				</Navbar.Collapse>
			</Navbar>
			<div>
				{navBarOption === "JsonEditor" && <JSONEditor />}
				{navBarOption === "Login" && <div> <h>to be implemented</h> </div>}
			</div>
		</div>
	);
};
