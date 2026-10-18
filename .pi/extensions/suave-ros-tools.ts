import { execFile } from "node:child_process";
import { promisify } from "node:util";

import { Type } from "typebox";
import type { ExtensionAPI } from "@earendil-works/pi-coding-agent";

const execFileAsync = promisify(execFile);
const DEFAULT_CONTAINER = process.env.SUAVE_ROS_CONTAINER ?? process.env.ROS_CHECK_CONTAINER ?? "suave";
const DEFAULT_WORKSPACE = process.env.SUAVE_ROS_WORKSPACE ?? process.env.ROS_CHECK_WORKSPACE ?? "/home/ubuntu-user/suave_ws";
const MAX_OUTPUT_CHARS = 16000;

type LaunchCheckOptions = {
	container?: string;
	workspace?: string;
	pkg: string;
	launchFile: string;
	launchArgs?: string[];
	timeoutSeconds?: number;
};

type ChangedCheckOptions = {
	container?: string;
	workspace?: string;
	runTests?: boolean;
	runLinters?: boolean;
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

async function dockerExec(container: string, command: string, timeoutMs = 10 * 60 * 1000) {
	const displayCommand = `docker exec ${container} bash -lc ${shellQuote(command)}`;
	try {
		const result = await execFileAsync("docker", ["exec", container, "bash", "-lc", command], {
			maxBuffer: 8 * 1024 * 1024,
			timeout: timeoutMs,
		});
		return {
			command: displayCommand,
			ok: true,
			output: truncateOutput(`${result.stdout}${result.stderr ? `\n${result.stderr}` : ""}`),
		};
	} catch (error) {
		const err = error as Error & { stdout?: string; stderr?: string; code?: number | string };
		const output = `${err.stdout || ""}${err.stderr ? `\n${err.stderr}` : ""}` || err.message;
		return {
			command: displayCommand,
			ok: false,
			output: truncateOutput(`command failed${err.code !== undefined ? ` with code ${err.code}` : ""}\n${output}`),
		};
	}
}

function rosSetupPrefix(workspace: string): string[] {
	return [
		`cd ${shellQuote(workspace)}`,
		"source /opt/ros/humble/setup.bash",
		"source install/setup.bash",
	];
}

async function runLaunchCheck(options: LaunchCheckOptions) {
	const container = options.container || DEFAULT_CONTAINER;
	const workspace = options.workspace || DEFAULT_WORKSPACE;
	const timeoutSeconds = options.timeoutSeconds ?? 20;
	const args = (options.launchArgs || []).map(shellQuote).join(" ");
	const launchCommand = [
		"timeout",
		`${timeoutSeconds}s`,
		"ros2",
		"launch",
		shellQuote(options.pkg),
		shellQuote(options.launchFile),
		args,
		"--show-args",
	].filter(Boolean).join(" ");
	const command = [...rosSetupPrefix(workspace), launchCommand].join(" && ");
	return dockerExec(container, command, (timeoutSeconds + 20) * 1000);
}

function parseLaunchCommandArgs(args: string): LaunchCheckOptions | undefined {
	const tokens = args.split(/\s+/).filter(Boolean);
	if (tokens.length < 2) {
		return undefined;
	}

	const options: LaunchCheckOptions = {
		container: DEFAULT_CONTAINER,
		workspace: DEFAULT_WORKSPACE,
		pkg: tokens[0],
		launchFile: tokens[1],
		launchArgs: [],
	};

	for (let i = 2; i < tokens.length; i += 1) {
		const token = tokens[i];
		if (token === "--container" && tokens[i + 1]) {
			options.container = tokens[i + 1];
			i += 1;
		} else if (token === "--workspace" && tokens[i + 1]) {
			options.workspace = tokens[i + 1];
			i += 1;
		} else if (token === "--timeout" && tokens[i + 1]) {
			options.timeoutSeconds = Number(tokens[i + 1]);
			i += 1;
		} else {
			options.launchArgs?.push(token);
		}
	}

	return options;
}

function parseChangedCommandArgs(args: string): ChangedCheckOptions {
	const options: ChangedCheckOptions = {
		container: DEFAULT_CONTAINER,
		workspace: DEFAULT_WORKSPACE,
		runTests: true,
		runLinters: true,
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
		} else if (token === "--no-tests") {
			options.runTests = false;
		} else if (token === "--no-linters") {
			options.runLinters = false;
		}
	}
	return options;
}

async function runChangedCheck(options: ChangedCheckOptions) {
	const container = options.container || DEFAULT_CONTAINER;
	const workspace = options.workspace || DEFAULT_WORKSPACE;
	const script = String.raw`
set -e
cd ${shellQuote(workspace)}
source /opt/ros/humble/setup.bash
source install/setup.bash
cd src/suave
changed=$(git status --short | awk '{print $2}')
if [ -z "$changed" ]; then
  echo "No changed files detected in src/suave."
  exit 0
fi
printf 'Changed files:\n%s\n' "$changed"
py_files=$(printf '%s\n' "$changed" | grep -E '\.py$' || true)
if [ ${options.runLinters === false ? "0" : "1"} -eq 1 ] && [ -n "$py_files" ]; then
  printf '\nRunning Python file checks...\n'
  python3 -m py_compile $py_files
  python3 -m flake8 $py_files
  python3 -m pydocstyle $py_files
fi
packages=$(printf '%s\n' "$changed" | awk -F/ '
  $1 ~ /^suave($|_)/ { print $1 }
  $1 == "suave" { print "suave" }
' | sort -u | tr '\n' ' ')
if [ ${options.runTests === false ? "0" : "1"} -eq 1 ]; then
  if [ -z "$packages" ]; then
    echo "No ROS package changes inferred; skipping colcon test."
  else
    printf '\nRunning colcon tests for packages: %s\n' "$packages"
    cd ${shellQuote(workspace)}
    colcon test --packages-select $packages --event-handlers console_direct+
  fi
fi
`;
	return dockerExec(container, script);
}

const rosLaunchCheckTool = {
	name: "ros_launch_check",
	label: "ROS Launch Check",
	description: "Validate a ROS 2 launch file inside the SUAVE Docker container using ros2 launch --show-args.",
	parameters: Type.Object({
		container: Type.Optional(Type.String({ description: "Docker container name. Defaults to suave or SUAVE_ROS_CONTAINER." })),
		workspace: Type.Optional(Type.String({ description: "Workspace path inside the container. Defaults to /home/ubuntu-user/suave_ws." })),
		pkg: Type.String({ description: "ROS package name containing the launch file." }),
		launchFile: Type.String({ description: "Launch file name, for example simulation.launch.py." }),
		launchArgs: Type.Optional(Type.Array(Type.String(), { description: "Optional launch arguments such as enable_water_current:=true." })),
		timeoutSeconds: Type.Optional(Type.Number({ description: "Timeout for ros2 launch --show-args. Default 20." })),
	}),
	async execute(_toolCallId, params) {
		const result = await runLaunchCheck(params);
		return {
			content: [{ type: "text", text: `$ ${result.command}\n\n${result.output}` }],
			details: result,
		};
	},
};

const changedCheckTool = {
	name: "changed_check",
	label: "Changed Check",
	description: "Infer changed SUAVE packages and run focused ROS checks inside Docker.",
	parameters: Type.Object({
		container: Type.Optional(Type.String({ description: "Docker container name. Defaults to suave or SUAVE_ROS_CONTAINER." })),
		workspace: Type.Optional(Type.String({ description: "Workspace path inside the container. Defaults to /home/ubuntu-user/suave_ws." })),
		runLinters: Type.Optional(Type.Boolean({ description: "Run py_compile, flake8, and pydocstyle for changed Python files. Default true." })),
		runTests: Type.Optional(Type.Boolean({ description: "Run colcon tests for inferred changed packages. Default true." })),
	}),
	async execute(_toolCallId, params) {
		const result = await runChangedCheck(params);
		return {
			content: [{ type: "text", text: `$ ${result.command}\n\n${result.output}` }],
			details: result,
		};
	},
};

export default function suaveRosToolsExtension(pi: ExtensionAPI) {
	pi.registerTool(rosLaunchCheckTool);
	pi.registerTool(changedCheckTool);
	pi.registerCommand("ros-launch-check", {
		description: "Validate a launch file in Docker. Usage: /ros-launch-check <pkg> <launch.py> [arg:=value ...]",
		handler: async (args, ctx) => {
			const options = parseLaunchCommandArgs(args);
			if (!options) {
				ctx.ui.notify("Usage: /ros-launch-check <pkg> <launch.py> [arg:=value ...]", "warning");
				return;
			}
			const result = await runLaunchCheck(options);
			ctx.ui.notify(`ros-launch-check finished. Command: ${result.command}`, result.ok ? "info" : "error");
			ctx.ui.notify(result.output.slice(-1000), result.ok ? "info" : "error");
		},
	});
	pi.registerCommand("changed-check", {
		description: "Run checks for changed SUAVE files in Docker. Usage: /changed-check [--no-tests] [--no-linters]",
		handler: async (args, ctx) => {
			const result = await runChangedCheck(parseChangedCommandArgs(args));
			ctx.ui.notify(`changed-check finished. Command: ${result.command}`, result.ok ? "info" : "error");
			ctx.ui.notify(result.output.slice(-1000), result.ok ? "info" : "error");
		},
	});
}
