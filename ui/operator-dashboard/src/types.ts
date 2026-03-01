export type Severity = "INFO" | "WARNING" | "ERROR";

export interface UserMessage {
  severity: Severity;
  code: string;
  text: string;
  stampSec: number;
}
