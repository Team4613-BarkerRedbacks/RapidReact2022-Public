package redbacks.robot.subsystems.drivetrain;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Translation2d;

public class SwerveModuleSet<PositionsT> {
	private final int nModules;
	
	private final SwerveModule[] modules;
	private final Translation2d[] offsets;
	private final Map<PositionsT, Integer> indexMap;
	
	public SwerveModuleSet(Supplier<SwerveModule> moduleSupplier, PositionsT[] positions, Function<PositionsT, Translation2d> positionToOffset) {
		this.nModules = positions.length;
		
		this.modules = new SwerveModule[nModules];
		this.offsets = new Translation2d[nModules];
		
		this.indexMap = new HashMap<PositionsT, Integer>();
		
		for(int i = 0; i < nModules; i++) {
			this.modules[i] = moduleSupplier.get();
			this.offsets[i] = positionToOffset.apply(positions[i]);
			
			this.indexMap.put(positions[i], i);
		}
	}
	
	public SwerveModule getModule(PositionsT position) {
		return modules[indexMap.get(position)];
	}
	
	public Translation2d[] getOffsets() {
		return offsets;
	}
	
	public void forEach(Consumer<SwerveModule> moduleConsumer) {
		for(SwerveModule module : modules) moduleConsumer.accept(module);
	}
	
	public void forEach(BiConsumer<SwerveModule, Integer> moduleConsumer) {
		for(int i = 0; i < nModules; i++) moduleConsumer.accept(modules[i], i);
	}
	
	public int size() {
		return nModules;
	}
}
