package org.usfirst.frc.team3042.robot.vision;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;

import org.usfirst.frc.team3042.lib.Translation2d;

public class TargetTracker {
	
	public static class TrackReport {
		public Translation2d fieldToTarget;
		
		public double latestTimestamp;
		
		public double stability;
		
		public int id;
		
		public TrackReport(TargetTrack track) {
			this.fieldToTarget = track.getSmoothedPosition();
			this.latestTimestamp = track.getLatestTimestamp();
			this.stability = track.getStability();
			this.id = track.getId();
		}
	}
	
	List<TargetTrack> currentTracks = new ArrayList<>();
	int nextId = 0;
	
	public TargetTracker() {
		
	}
	
	public void reset() {
		currentTracks.clear();
	}
	
	public void update(double timestamp, List<Translation2d> fieldToTargets) {
		boolean hasUpdatedTrack = false;
		// Try to update existing tracks
		for (Translation2d target : fieldToTargets) {
			for (TargetTrack track : currentTracks) {
				if (!hasUpdatedTrack) {
					if (track.tryUpdate(timestamp, target)) {
						hasUpdatedTrack = true;
					}
				} else {
					track.pruneByTime();
				}
			}
		}
		
		// Prune dead tracks
		for (Iterator<TargetTrack> it = currentTracks.iterator(); it.hasNext();) {
			TargetTrack track = it.next();
			if (!track.isAlive()) {
				it.remove();
			}
		}
		
		// If all tracks are dead, start new ones
		if (currentTracks.isEmpty()) {
			for (Translation2d target : fieldToTargets) {
				currentTracks.add(new TargetTrack(timestamp, target, nextId));
				nextId++;
			}
		}
	}
	
	public boolean hasTracks() {
		return !currentTracks.isEmpty();
	}
	
	public List<TrackReport> getTracks() {
		List<TrackReport> reports = new ArrayList<>();
		for (TargetTrack track : currentTracks) {
			reports.add(new TrackReport(track));
		}
		
		return reports;
	}

}
