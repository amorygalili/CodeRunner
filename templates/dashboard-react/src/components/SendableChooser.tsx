import { useNTValue } from "../coderunner/hooks";

/**
 * A WPILib `SendableChooser` (e.g. an autonomous picker published with
 * `SmartDashboard.putData("Auto Choices", chooser)`).
 */
export function SendableChooser({
	ntKey,
	label,
}: {
	/** The chooser's table, e.g. "/SmartDashboard/Auto Choices". */
	ntKey: string;
	label?: string;
}) {
	const [options] = useNTValue<string[]>(`${ntKey}/options`, []);
	const [defaultValue] = useNTValue<string>(`${ntKey}/default`, "");
	const [active] = useNTValue<string>(`${ntKey}/active`, "");
	const [selected, setSelected] = useNTValue<string>(`${ntKey}/selected`, "");

	const current = selected || active || defaultValue;
	// The robot copies `selected` into `active` once it has seen the change.
	const pending = selected !== "" && selected !== active;

	return (
		<label className="field">
			<span className="field-label">{label ?? ntKey.split("/").pop()}</span>
			<select
				value={current}
				disabled={options.length === 0}
				onChange={(event) => setSelected(event.target.value, "string")}
			>
				{options.length === 0 && <option value="">No options published</option>}
				{options.map((option) => (
					<option key={option} value={option}>
						{option}
					</option>
				))}
			</select>
			{pending && <span className="hint">Waiting for the robot…</span>}
		</label>
	);
}
