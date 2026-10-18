import { execFile } from "node:child_process";
import { promisify } from "node:util";

import { Type } from "typebox";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

const execFileAsync = promisify(execFile);
const DEFAULT_CONTAINER = process.env.SUAVE_ROS_CONTAINER ?? process.env.ROS_CHECK_CONTAINER ?? "suave";
const DEFAULT_WORKSPACE = process.env.SUAVE_ROS_WORKSPACE ?? process.env.ROS_CHECK_WORKSPACE ?? "/home/ubuntu-user/suave_ws";
const MAX_OUTPUT_CHARS = 16000;

type RosCheckOptions = {
	container?: string;
	workspace?: string;
	packages?: string[];
	files?: string[];
	runLinters?: boolean;
	runTests?: boolean;
};

function shellQuote(value: string): string {
	return `'${value.replace(/'/g, `'"'"'`)}'`;
}

function truncateOutput(output: string): string {
	if (output.length <= MAX_OUTPUT_CHARS) {
		return output;
	}

	return `${output.slice(0, 4000)}\n\n... output truncated ...\n\n${output.slice(-12000)}`;
}

function parseCommandArgs(args: string): RosCheckOptions {
	const options: RosCheckOptions = {
		container: DEFAULT_CONTAINER,
		workspace: DEFAULT_WORKSPACE,
		packages: [],
		files: [],
		runLinters: true,
		runTests: true,
	};

	const tokens = args.split(/\s+/).filter(Boolean);
	for (let i = 0; i < tokens.length; i += 1) {
		const token = tokens[i];
		if (token === "--container" && tokens[i + 1]) {
			options.container = tokens[i + 1];
			i += 1;
		} else if (token === "--workspace" && tokens[i + 1]) {
			options.workspace = tokens[i + 1];
			i += 1;
		} else if (token === "--file" && tokens[i + 1]) {
			options.files?.push(tokens[i + 1]);
			i += 1;
		} else if (token === "--no-linters") {
			options.runLinters = false;
		} else if (token === "--no-tests") {
			options.runTests = false;
		} else {
			options.packages?.push(token);
		}
	}

	return options;
}

function buildRosCommand(options: RosCheckOptions): string {
	const workspace = options.workspace || DEFAULT_WORKSPACE;
	const files = options.files || [];
	const packages = options.packages || [];
	const commands = [
		`cd ${shellQuote(workspace)}`,
		"source /opt/ros/humble/setup.bash",
		"source install/setup.bash",
	];

	if (options.runLinters !== false && files.length > 0) {
		const quotedFiles = files.map(shellQuote).join(" ");
		commands.push(`python3 -m py_compile ${quotedFiles}`);
		commands.push(`python3 -m flake8 ${quotedFiles}`);
		commands.push(`python3 -m pydocstyle ${quotedFiles}`);
	}

	if (options.runTests !== false) {
		const packageList = packages.length > 0 ? packages : ["suave"];
		commands.push(
			`colcon test --packages-select ${packageList.map(shellQuote).join(" ")} --event-handlers console_direct+`,
		);
	}

	return commands.join(" && ");
}

async function runRosCheck(options: RosCheckOptions): Promise<{ command: string; output: string }> {
	const container = options.container || DEFAULT_CONTAINER;
	const command = buildRosCommand(options);
	try {
		const result = await execFileAsync("docker", ["exec", container, "bash", "-lc", command], {
			maxBuffer: 8 * 1024 * 1024,
			timeout: 10 * 60 * 1000,
		});
		return {
			command: `docker exec ${container} bash -lc ${shellQuote(command)}`,
			output: truncateOutput(`${result.stdout}${result.stderr ? `\n${result.stderr}` : ""}`),
		};
	} catch (error) {
		const err = error as Error & { stdout?: string; stderr?: string; code?: number | string };
		const output = `${err.stdout || ""}${err.stderr ? `\n${err.stderr}` : ""}` || err.message;
		return {
			command: `docker exec ${container} bash -lc ${shellQuote(command)}`,
			output: truncateOutput(`ros-check failed${err.code !== undefined ? ` with code ${err.code}` : ""}\n${output}`),
		};
	}
}

const rosCheckTool = {
	name: "ros_check",
	label: "ROS Check",
	description: "Run SUAVE ROS validation commands inside the configured Docker container.",
	parameters: Type.Object({
		container: Type.Optional(Type.String({ description: "Docker container name. Defaults to suave or SUAVE_ROS_CONTAINER." })),
		workspace: Type.Optional(Type.String({ description: "Workspace path inside the container. Defaults to /home/ubuntu-user/suave_ws." })),
		packages: Type.Optional(Type.Array(Type.String(), { description: "ROS package names to test with colcon." })),
		files: Type.Optional(Type.Array(Type.String(), { description: "Files to check with py_compile, flake8, and pydocstyle." })),
		runLinters: Type.Optional(Type.Boolean({ description: "Run py_compile, flake8, and pydocstyle for files. Default true." })),
		runTests: Type.Optional(Type.Boolean({ description: "Run colcon test. Default true." })),
	}),
	async execute(_toolCallId, params) {
		const result = await runRosCheck(params);
		return {
			content: [{ type: "text", text: `$ ${result.command}\n\n${result.output}` }],
			details: result,
		};
	},
};

export default function suaveRosCheckExtension(pi: ExtensionAPI) {
	pi.registerTool(rosCheckTool);
	pi.registerCommand("ros-check", {
		description: "Run SUAVE ROS checks in Docker. Usage: /ros-check [package ...] [--file path] [--no-tests] [--no-linters]",
		handler: async (args, ctx) => {
			const result = await runRosCheck(parseCommandArgs(args));
			ctx.ui.notify(`ros-check finished. Command: ${result.command}`, "info");
			ctx.ui.notify(result.output.slice(-1000), result.output.includes("failed") ? "error" : "info");
		},
	});
}
