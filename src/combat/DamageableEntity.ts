// DamageableEntity — per-zone hit points for an aircraft (or any damageable
// target). Ported from the v2 template's Aircraft.damage() zone logic
// (pandora-template.html ~1134): wing hits x1.2, engine hits x1.5, smoke
// under 60% hp, fire under 35% — those *effects* are applied by the
// Aircraft's solver damage input, not here. This class only tracks hp and
// answers "what broke / who gets the kill".
//
// v1.1 rule (kept): kills/score ONLY credit when the last hit came from the
// player. Every applyDamage() call records lastHitByIsPlayer, so a kill is
// credited only if the final blow was the player's.

export interface DamageResult {
  /** True when this hit destroyed the entity. */
  destroyed: boolean;
  /** Zone whose hp hit zero on this hit (first time only), or null. */
  zoneLost: string | null;
  /** True when destroyed AND the last hit was by the player. */
  killedByPlayer: boolean;
}

/** Zone damage multipliers, from the template. */
const ZONE_MULT: Record<string, number> = { wing: 1.2, engine: 1.5 };

export class DamageableEntity {
  private readonly zoneMax: Record<string, number>;
  private readonly zoneHp: Record<string, number>;
  private readonly lostZones = new Set<string>();
  private lastHitByIsPlayer = false;
  private destroyedFlag = false;

  /** zones: max hp per zone, e.g. { fuselage: 100, wing: 60, engine: 60 }. */
  constructor(zones: Record<string, number>) {
    this.zoneMax = { ...zones };
    this.zoneHp = { ...zones };
  }

  applyDamage(amount: number, zone: string, byPlayer: boolean): DamageResult {
    this.lastHitByIsPlayer = byPlayer;
    if (this.destroyedFlag) {
      return { destroyed: true, zoneLost: null, killedByPlayer: byPlayer };
    }
    const mult = ZONE_MULT[zone] ?? 1;
    const key = zone in this.zoneHp ? zone : 'fuselage';
    if (!(key in this.zoneHp)) {
      // No zones at all (should not happen) — treat as destroyed on any hit.
      this.destroyedFlag = true;
      return { destroyed: true, zoneLost: null, killedByPlayer: byPlayer };
    }
    this.zoneHp[key] = Math.max(0, this.zoneHp[key] - amount * mult);

    let zoneLost: string | null = null;
    if (this.zoneHp[key] <= 0 && !this.lostZones.has(key)) {
      this.lostZones.add(key);
      zoneLost = key;
    }

    // Destroyed when the fuselage zone is lost (if one exists), or when
    // every zone is gone / total hp is exhausted.
    const total = this.hp;
    if ((key === 'fuselage' && this.zoneHp[key] <= 0) || total <= 0) {
      this.destroyedFlag = true;
    }
    return {
      destroyed: this.destroyedFlag,
      zoneLost,
      killedByPlayer: this.destroyedFlag && this.lastHitByIsPlayer,
    };
  }

  /** Sum of remaining hp across all zones. */
  get hp(): number {
    return Object.values(this.zoneHp).reduce((a, b) => a + b, 0);
  }

  /** Sum of max hp across all zones. */
  get maxHp(): number {
    return Object.values(this.zoneMax).reduce((a, b) => a + b, 0);
  }

  /** Remaining hp of one zone (0 when the zone does not exist). */
  hpFor(zone: string): number {
    return this.zoneHp[zone] ?? 0;
  }

  /** Max hp of one zone (0 when the zone does not exist). */
  maxHpFor(zone: string): number {
    return this.zoneMax[zone] ?? 0;
  }

  /** True once a zone has been lost (hp reached zero). */
  isZoneLost(zone: string): boolean {
    return this.lostZones.has(zone);
  }

  isDestroyed(): boolean {
    return this.destroyedFlag;
  }

  /** True when the most recent hit was by the player (kill-credit source). */
  wasLastHitByPlayer(): boolean {
    return this.lastHitByIsPlayer;
  }

  /** Full repair (respawn). */
  repair(): void {
    for (const k of Object.keys(this.zoneHp)) this.zoneHp[k] = this.zoneMax[k];
    this.lostZones.clear();
    this.lastHitByIsPlayer = false;
    this.destroyedFlag = false;
  }
}
